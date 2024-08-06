mod file_times;
mod csv_reader;
mod binary_reader;
mod filtering;

use binary_reader::AnalogData;
use file_times::AudioSamples;
use pyo3::{prelude::*};
use sci_rs::signal::filter::sosfiltfilt_dyn;


#[pyfunction]
fn read_saleae_csv(filename: &str, num_channels: i32) -> PyResult<AudioSamples> {
    let reading = csv_reader::read_csv(filename, num_channels);
    Ok(reading)
}

#[pyfunction]
fn read_binary_file(filename: &str) -> PyResult<AnalogData> {
    let reading = binary_reader::read_binary_file(filename).ok().unwrap();
    Ok(reading)
}

#[pyfunction]
fn read_binary_folder(directory: &str) -> PyResult<AudioSamples> {
   let reading = binary_reader::read_binary_folder(directory).ok().unwrap();
    Ok(reading)
}

#[pyfunction]
fn filter_samples(data: &AudioSamples, lowcut: f64, highcut: f64, fs: f64) -> Vec::<Vec<f64>>{
    let mut filtered_channels = Vec::<Vec<f64>>::new();

    for channel in &data.channels {
        let filter = filtering::gen_filter(lowcut, highcut, fs);
        let channel_as_f64 = channel.iter().map(|x| *x as f64).collect::<Vec<f64>>();
        let mut filtered = sosfiltfilt_dyn(channel_as_f64.iter(), &filter.sos);
        filtered.iter_mut().for_each(|x| *x = x.abs());
        filtered_channels.push(filtered);
    }

    filtered_channels
}

#[pyfunction]
fn get_time_difs(samples: &AudioSamples, lowcut: f64, highcut: f64, fs: usize, time_skip_after_peak: f64) ->  Vec<Vec<f64>>{
    let difs = filtering::get_time_difs(samples, lowcut, highcut, fs, time_skip_after_peak);
    difs
}

fn get_spike_amplitude(data: &Vec<f64>, peak: usize, fs: usize, spike_time: f64) -> f64 {
    let spike_samples = (fs as f64 * spike_time) as usize;

    let subsection = &data[peak-spike_samples..peak+spike_samples];
    let max = subsection.iter().reduce(|x, y| if x > y {x} else {y}).unwrap();

    *max
}

#[pyclass]
struct PipelineConfig {
    #[pyo3(get)]
    #[pyo3(set)]
    lowcut: f64,

    #[pyo3(get)]
    #[pyo3(set)]
    highcut: f64,

    #[pyo3(get)]
    #[pyo3(set)]
    spike_time_match_radius: f32,

    #[pyo3(get)]
    #[pyo3(set)]
    time_skip_after_peak: f64,

    #[pyo3(get)]
    #[pyo3(set)]
    spike_duration: f64,

    #[pyo3(get)]
    #[pyo3(set)]
    noise_floor_stdev_mult: f64,
}

#[pymethods]
impl PipelineConfig {
    #[new]
    #[pyo3(signature = (
        lowcut, 
        highcut, 
        spike_time_match_radius=0.005, 
        time_skip_after_peak=1.0, 
        spike_duration=0.005536, 
        noise_floor_stdev_mult=5.0))]
    fn new(lowcut: f64, highcut: f64, spike_time_match_radius: f32, time_skip_after_peak: f64, spike_duration: f64, noise_floor_stdev_mult: f64) -> Self {
        PipelineConfig {
            lowcut,
            highcut,
            spike_time_match_radius,
            time_skip_after_peak,
            spike_duration,
            noise_floor_stdev_mult,
        }
    }
}


#[pyclass]
struct Spike {
    #[pyo3(get)]
    time: f64,
    #[pyo3(get)]
    amplitude: f64,
}

#[pyfunction]
fn get_spike_times(samples: &AudioSamples, config: &PipelineConfig, fs: usize) -> Vec::<Vec<Spike>>{
    let filter = filtering::gen_filter(config.lowcut,config.highcut, fs as f64);

    let mut peaks = Vec::<Vec<Spike>>::new();

    for i in 0..samples.channels.len() {
        let channel_as_f64 = samples.channels[i].iter().map(|x| *x as f64).collect::<Vec<f64>>();
        let pks = filtering::get_spike_starts(&channel_as_f64, fs, &filter, config.time_skip_after_peak, config.noise_floor_stdev_mult);

        let mut spikes = Vec::<Spike>::new();
        for &pk in &pks {
            let time = samples.times[pk] as f64;
            let amplitude = get_spike_amplitude(&channel_as_f64, pk, fs, config.spike_duration);
            spikes.push(Spike{time, amplitude});
        }

        // let pks = pks.iter().map(|&x| samples.times[x] as f64).collect::<Vec<f64>>();
        peaks.push(spikes);
    }

    peaks
}

#[pyfunction]
fn test_arr_len_marshalling(length: usize) -> Vec<usize> {
    let mut arr = Vec::<usize>::new();
    for i in 0..length {
        arr.push(i);
    }
    arr
}

fn match_times(times: &Vec<Vec<f64>>, dt: f32) -> Vec<Vec<Option<f64>>> {
    let mut groupings = Vec::<Vec<Option<f64>>>::new();
    let mut indices = Vec::<usize>::with_capacity(times.len());
    for _ in 0..times.len() { indices.push(0);}

    loop {
        let mut ind_times = Vec::<f64>::with_capacity(times.len());
        for i in 0..times.len() {
            ind_times.push(times[i][indices[i]]);
        }

        let mut min_time = ind_times[0];
        for i in 1..ind_times.len() {
            if ind_times[i] < min_time { min_time = ind_times[i]; }
        }

        let mut group = Vec::<Option<f64>>::with_capacity(ind_times.len());
        for _ in 0..ind_times.len() { group.push(None); }

        for i in 0..ind_times.len() {
            if indices[i] == times[i].len() { continue; }

            if times[i][indices[i]] - min_time < dt as f64 {
                group[i] = Some(times[i][indices[i]]);
                indices[i] += 1;
            }
        }

        groupings.push(group);

        if indices.iter().enumerate().all(|(i, x)| *x == times[i].len()) { break; }
    }

    groupings

}

fn what_comes_first(groupings: &Vec<Vec<Option<f64>>>) -> Vec<usize>{
    let mut firsts = Vec::<usize>::with_capacity(groupings.len());

    for group in groupings {
        let mut min_val: f64 = 1e10;
        let mut min_index = 0;
        for i in 0..group.len() {
            if group[i].is_some() && group[i].unwrap() < min_val {
                min_val = group[i].unwrap();
                min_index = i;
            }
        }

        firsts.push(min_index);
    }

    firsts
}


#[pyfunction]
fn spikes_pipeline(directory: &str, config: &PipelineConfig) -> Vec<Vec<Spike>> {
    let readings = read_binary_folder(directory).ok().unwrap();
    let spikes = get_spike_times(&readings, config, readings.sample_rate as usize);
    spikes
}

#[pyfunction]
fn get_first_hydrophones(spike_groups: Vec<Vec<PyRef<Spike>>>, config: &PipelineConfig) -> Vec<usize> {
    let times = spike_groups.iter().map(|x| x.iter().map(|y| y.time).collect::<Vec<f64>>()).collect::<Vec<Vec<f64>>>();
    let gs: Vec<Vec<Option<f64>>> = match_times(&times, config.spike_time_match_radius);
    let firsts = what_comes_first(&gs);

    firsts
}

#[pyfunction]
fn pipeline(directory: &str, config: &PipelineConfig) -> Vec<usize> {
    let readings = read_binary_folder(directory).ok().unwrap();

    let spikes = get_spike_times(&readings, config, readings.sample_rate as usize);
    let times = spikes.iter().map(|x| x.iter().map(|y| y.time).collect::<Vec<f64>>()).collect::<Vec<Vec<f64>>>();
    let gs = match_times(&times, config.spike_time_match_radius);
    let firsts = what_comes_first(&gs);

    firsts
}

/// A Python module implemented in Rust.
#[pymodule]
fn pinger2(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(read_saleae_csv, m)?)?;
    m.add_function(wrap_pyfunction!(read_binary_file, m)?)?;
    m.add_function(wrap_pyfunction!(read_binary_folder, m)?)?;
    m.add_function(wrap_pyfunction!(filter_samples, m)?)?;
    m.add_function(wrap_pyfunction!(get_time_difs, m)?)?;
    m.add_function(wrap_pyfunction!(get_spike_times, m)?)?;
    m.add_function(wrap_pyfunction!(test_arr_len_marshalling, m)?)?;
    m.add_function(wrap_pyfunction!(pipeline, m)?)?;
    m.add_function(wrap_pyfunction!(spikes_pipeline, m)?)?;
    m.add_function(wrap_pyfunction!(get_first_hydrophones, m)?)?;

    m.add_class::<PipelineConfig>()?;
    Ok(())
}
