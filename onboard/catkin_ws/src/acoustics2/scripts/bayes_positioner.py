#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 17 15:27:04 2019

@author: bmoseley
"""

import numpy as np
import pymc3 as pm
from scipy.stats import gaussian_kde



class BayesianTDOAPositioner:
    """Class for carrying out Bayesian TDOA positioning.
    Requires at least 4 stations (to solve for x,y,v,t1).
    """
    
    def __init__(self,
                 stations,
                 x_lim=1000,# maximum box size (m)
                 v_mu=346,# mean of velocity prior (m/s)
                 v_sd=20,# standard deviation of velocity prior (m/s)
                 t_sd=0.05):# standard deviation of observed values (s)

        t_lim = np.sqrt(2)*x_lim/v_mu# resulting max tdoa value, used for t1 limit (s)
        
        # check if well posed
        if len(stations)<4:
            print("WARNING: at least 4 stations usually required for bayesian tdoa positioning!")
        
        # assert bounding box is big enough
        if np.max(stations) > 0.75*x_lim or np.min(stations) < 0.25*x_lim:
            raise Exception("ERROR: stations are not within bounding box!")
        
        self.x_lim = x_lim
        self.v_mu = v_mu
        self.v_sd = v_sd
        self.t_sd = t_sd
        self.t_lim = t_lim
        self.stations = stations
        
    def sample(self, tdoa, draws=2000, tune=2000, chains=4, init='jitter+adapt_diag', verbose=True):
        "Carry out Bayesian inference"
        
        x_lim = self.x_lim
        v_mu = self.v_mu
        v_sd = self.v_sd
        t_sd = self.t_sd
        t_lim = self.t_lim
        stations = self.stations
        
        # assert correct number of observations
        if len(tdoa) != len(stations):
            raise Exception("ERROR: number of observations must match number of stations! (%i, %i)"%(len(tdoa), len(stations)))
        
        # assert max tdoa is not larger than t_lim
        if np.max(tdoa) > t_lim: 
            raise Exception("ERROR: tdoa > t_lim")
            
        with pm.Model():
        
            # Priors
            x = pm.Uniform("x", lower=0, upper=x_lim, shape=2)# prior on the source location (m)
            v = pm.Normal("v", mu=v_mu, sigma=v_sd)# prior on the wave speed (m/s)
            t1 = pm.Uniform("t1", lower=-0.5*t_lim, upper=t_lim)# prior on the time offset (s)
            
            # Physics
            d = pm.math.sqrt(pm.math.sum((stations - x)**2, axis=1))# distance between source and receivers
            t0 = d/v# time of arrival (TOA) of each receiver
            t = t0-t1# time difference of arrival (TDOA) from the time offset
            
            # Observations
            Y_obs = pm.Normal('Y_obs', mu=t, sd=t_sd, observed=tdoa)# we assume Gaussian noise on the TDOA measurements
            
            # Posterior sampling
            #step = pm.HamiltonianMC()
            trace = pm.sample(draws=draws, tune=tune, chains=chains, target_accept=0.95, init=init)#, step=step)# i.e. tune for 1000 samples, then draw 5000 samples
            
            summary = pm.summary(trace)
        
        mu = np.array(summary["mean"])
        sd = np.array(summary["sd"])
        
        if verbose:
            print("Percent divergent traces: %.2f %%"%(trace['diverging'].nonzero()[0].size / len(trace) * 100))
        
        return trace, summary, mu, sd
    
    def fit_xy_posterior(self, trace):
        """Helper function to estimate mu and sd of samples from a distribution,
        designed for when the tails of the distributions are large or non-zero"""
        
        # take mu to be the maximum of the posterior
        # take sigma to be the kde's half-width at 0.6065 (=normal distribution value at x=sigma)
        r = np.linspace(0, self.x_lim, 1000)
        ds = [gaussian_kde(trace['x'][:,i])(r) for i in range(2)]
        mu = [r[np.argmax(d)] for d in ds]
        widths = [r[d>0.6065*np.max(d)] for d in ds]
        sd = [(np.max(width)-np.min(width))/2. for width in widths]
        
        return mu, sd
    
    
    def forward(self, x, v=346):
        "predict time of flight for given source position"

        d = np.linalg.norm(self.stations-x, axis=1)
        t0 = d/v# time of flight values
        return t0
        
    
if __name__ == "__main__":
    
    import matplotlib
    import matplotlib.pyplot as plt
    print('Running on PyMC3 v{}'.format(pm.__version__))

    
    # generate some test data
    N_STATIONS = 4
    np.random.seed(1)
    stations = np.random.randint(250,750, size=(N_STATIONS,2))# station positions (m)
    x_true = np.array([500,500])# true source position (m)
    v_true = 346.# speed of sound (m/s)
    t1_true = 0.5*(np.sqrt(2)*500/346)# can be any constant, as long as it is within the uniform distribution prior on t1
    d_true = np.linalg.norm(stations-x_true, axis=1)
    t0_true = d_true/v_true# true time of flight values
    t_obs = t0_true-t1_true# true time difference of arrival values
    np.random.seed(1)
    t_obs = t_obs+0.05*np.random.randn(*t_obs.shape)# noisy observations
    
    # sample
    np.random.seed(1)
    B = BayesianTDOAPositioner(stations)
    trace, summary, _, _ = B.sample(t_obs)

    # analysis
    mu, sd = B.fit_xy_posterior(trace)
    t0_pred = B.forward(mu)

    # report
    print(summary)
    print(t0_true)
    print(t0_pred)
    print(t1_true)

    # trace plot
    ax,ay = pm.traceplot(trace, compact=False)[1:3,0]
    ax.hlines(0.6065*ax.get_ylim()[1], mu[0]-sd[0], mu[0]+sd[0])# add mu, sigma lines to x,y plots
    ay.hlines(0.6065*ay.get_ylim()[1], mu[1]-sd[1], mu[1]+sd[1])
    #plt.savefig("bayes_positioner_result1.jpg", bbox_inches='tight', pad_inches=0.01, dpi=300)
    #pm.autocorrplot(trace)
    #pm.plot_posterior(trace)

    # local map
    plt.figure(figsize=(5,5))
    plt.scatter(stations[:,0], stations[:,1], marker="^", s=80, label="Receivers")
    plt.scatter(x_true[0], x_true[1], s=40, label="True source position")
    ell = matplotlib.patches.Ellipse(xy=(mu[0], mu[1]),
              width=4*sd[0], height=4*sd[1],
              angle=0., color='black', label="Posterior ($2\sigma$)", lw=1.5)
    ell.set_facecolor('none')
    plt.gca().add_patch(ell)
    plt.legend(loc=2)
    plt.xlim(250, 750)
    plt.ylim(250, 750)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    #plt.savefig("bayes_positioner_result2.jpg", bbox_inches='tight', pad_inches=0.01, dpi=300)
    plt.show()
    