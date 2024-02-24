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

pub fn get_spike_starts(data: &Vec<f64>, sample_rate: usize, filter: &SosFormatFilter<f64>) -> Vec<usize> {
    let mut filtered = sosfiltfilt_dyn(data.iter(), &filter.sos);
    filtered.iter_mut().for_each(|x| *x = x.abs());

    let all_peaks_inds = find_peaks(&filtered, 0.0);
    let all_peaks = all_peaks_inds.iter().map(|&x| filtered[x]).collect::<Vec<f64>>();

    let mean = all_peaks.iter().sum::<f64>() / all_peaks.len() as f64;
    let mut stdev = all_peaks.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / all_peaks.len() as f64;
    stdev = stdev.sqrt();
    let thresh = mean + 5.0 * stdev;

    let mut thresh_peaks = Vec::<usize>::new();
    let mut sample = 0;
    while sample < filtered.len() {
        if filtered[sample] > thresh {
            while sample > 0 && filtered[sample-1] < filtered[sample] {
                sample -= 1;
            }
            thresh_peaks.push(sample);
            sample += sample_rate;
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
            sample += sample_rate;
        }
        sample += 1;
    }

    let mut sustained_peaks = Vec::<usize>::new();
    for i in 0..thresh_peaks.len()-1 {
        if thresh_peaks[i+1] - thresh_peaks[i] > 2 * sample_rate {
            sustained_peaks.push(thresh_peaks[i]);
        }
    }

    sustained_peaks

}

pub fn get_time_difs(samples: &AudioSamples, lowcut: f64, highcut: f64, fs: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {

    let filter = gen_filter(lowcut, highcut, fs);
    let ch1_peaks = get_spike_starts(&samples.channel1, 625_000, &filter);
    let ch2_peaks = get_spike_starts(&samples.channel2, 625_000, &filter);
    let ch3_peaks = get_spike_starts(&samples.channel3, 625_000, &filter);

    let ch1_peaks = ch1_peaks.iter().map(|&x| samples.times[x]).collect::<Vec<f64>>();
    let ch2_peaks = ch2_peaks.iter().map(|&x| samples.times[x]).collect::<Vec<f64>>();
    let ch3_peaks = ch3_peaks.iter().map(|&x| samples.times[x]).collect::<Vec<f64>>();

    let ch1_minus_ch2 = ch1_peaks.iter().zip(ch2_peaks.iter()).map(|(x,y)| x-y).collect::<Vec<f64>>();
    let ch1_minus_ch3 = ch1_peaks.iter().zip(ch3_peaks.iter()).map(|(x,y)| x-y).collect::<Vec<f64>>();
    let ch2_minus_ch3 = ch2_peaks.iter().zip(ch3_peaks.iter()).map(|(x,y)| x-y).collect::<Vec<f64>>();

    (ch1_minus_ch2, ch1_minus_ch3, ch2_minus_ch3)
}

fn get_pinger_location(pinger_x: f64, pinger_y: f64, bot_angle: f64) -> (f64, f64) {
    let x = pinger_x * bot_angle.cos() - pinger_y * bot_angle.sin();
    let y = pinger_x * bot_angle.sin() + pinger_y * bot_angle.cos();
    (x, y)
}


