use std::iter::Sum;
use sci_rs::signal::filter::{design::*, sosfiltfilt_dyn};
use num_traits::Float;
use crate::file_times::AudioSamples;


pub fn gen_filter(lowcut: f64, highcut: f64, fs: f64) -> SosFormatFilter<f64> {
    let filter = iirfilter_dyn::<f64>(
        4,
        vec![lowcut, highcut],
        None,
        None,
        Some(FilterBandType::Bandpass),
        Some(FilterType::Butterworth),
        Some(false),
        Some(FilterOutputType::Sos),
        Some(fs),
    );
    let DigitalFilter::Sos(sos) = filter else { panic!("Not SOS filter") };
    sos
}

pub fn find_peaks<T: Float + Sum>(data: &[T], threshold: T) -> Vec<usize> {
    let mut peaks = Vec::<usize>::new();
    for i in 1..data.len()-1 {
        if data[i] > data[i-1] && data[i] > data[i+1] && data[i] > threshold {
            peaks.push(i);
        }
    }

    peaks
}

pub fn get_spike_starts(data: &Vec<f64>, sample_rate: usize, filter: &SosFormatFilter<f64>, timeskip: f64, stdev_mult: f64) -> Vec<usize> {
    let mut filtered = sosfiltfilt_dyn(data.iter(), &filter.sos);
    filtered.iter_mut().for_each(|x| *x = x.abs());

    let all_peaks_inds = find_peaks(&filtered, 0.0);
    let all_peaks = all_peaks_inds.iter().map(|&x| filtered[x]).collect::<Vec<f64>>();

    let mean = all_peaks.iter().sum::<f64>() / all_peaks.len() as f64;
    let mut stdev = all_peaks.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / all_peaks.len() as f64;
    stdev = stdev.sqrt();
    let thresh = mean + stdev_mult * stdev;

    let sample_skip = (timeskip * sample_rate as f64) as usize;

    let mut thresh_peaks = Vec::<usize>::new();
    let mut sample = 0;
    while sample < filtered.len() {
        if filtered[sample] > thresh {
            while sample > 0 && filtered[sample-1] < filtered[sample] {
                sample -= 1;
            }
            thresh_peaks.push(sample);
            sample += sample_skip;
        }
        sample += 1;
    }

    thresh_peaks
}

pub fn get_sustained_spikes(data: &Vec<f64>, sample_rate: f64, filter: &SosFormatFilter<f64>) -> Vec<usize> {
    let mut filtered = sosfiltfilt_dyn(data.iter(), &filter.sos);
    filtered.iter_mut().for_each(|x| *x = x.abs());

    let all_peaks_inds = find_peaks(&filtered, 0.0);
    let all_peaks = all_peaks_inds.iter().map(|&x| filtered[x]).collect::<Vec<f64>>();

    let mean = all_peaks.iter().sum::<f64>() / all_peaks.len() as f64;
    let mut stdev = all_peaks.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / all_peaks.len() as f64;
    stdev = stdev.sqrt();
    let thresh = mean + 2.0 * stdev;
    let time_thresh = sample_rate * 0.001;

    let ufs = sample_rate as usize;

    let mut thresh_peaks = Vec::<usize>::new();
    let mut sample = 0;
    while sample < filtered.len() {
        if filtered[sample] > thresh {
            let mut num_above = 0;
            let start = sample;
            while filtered[sample] > thresh {
                sample += 1;
                num_above += 1;
            }
            if num_above > time_thresh as usize {
                thresh_peaks.push(start);
            }

            thresh_peaks.push(sample);
            sample += ufs;
        }
        sample += 1;
    }

    let mut sustained_peaks = Vec::<usize>::new();
    for i in 0..thresh_peaks.len()-1 {
        if thresh_peaks[i+1] - thresh_peaks[i] > 2 * ufs {
            sustained_peaks.push(thresh_peaks[i]);
        }
    }

    sustained_peaks

}

pub fn get_time_difs(samples: &AudioSamples, lowcut: f64, highcut: f64, fs: usize, time_skip_after_peak: f64) -> Vec<Vec<f64>> {

    let filter = gen_filter(lowcut, highcut, fs as f64);

    let mut subs = Vec::<Vec<f64>>::new();
    let mut peaks = Vec::<Vec<f64>>::new();

    for i in 0..samples.channels.len() {
        let channel_as_f64 = samples.channels[i].iter().map(|x| *x as f64).collect::<Vec<f64>>();
        let pks = get_spike_starts(&channel_as_f64, fs, &filter, time_skip_after_peak, 5.0);
        let pks = pks.iter().map(|&x| samples.times[x] as f64).collect::<Vec<f64>>();
        peaks.push(pks);
    }

    for i in 0..samples.channels.len()-1 {
        for j in i+1..samples.channels.len() {
            let min = peaks[i].iter().zip(peaks[j].iter()).map(|(x,y)| x-y).collect::<Vec<f64>>();
            subs.push(min);
        }
    }

    subs
}

fn get_pinger_location(pinger_x: f64, pinger_y: f64, bot_angle: f64) -> (f64, f64) {
    let x = pinger_x * bot_angle.cos() - pinger_y * bot_angle.sin();
    let y = pinger_x * bot_angle.sin() + pinger_y * bot_angle.cos();
    (x, y)
}


