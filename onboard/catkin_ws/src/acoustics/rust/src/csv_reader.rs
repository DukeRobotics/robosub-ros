use std::fs;
use crate::file_times::AudioSamples;

pub fn read_csv(filename: &str, num_channels: i32) -> AudioSamples {
    let file_contents = fs::read_to_string(filename).unwrap();

    let mut times = Vec::<f32>::new();
    let mut channels = Vec::<Vec<f32>>::new();

    // let mut channel1 = Vec::<f64>::new();
    // let mut channel2 = Vec::<f64>::new();
    // let mut channel3 = Vec::<f64>::new();

    let mut lines = file_contents.lines();
    lines.next(); // skip header
    for line in lines {
        let mut fields = line.split(",");

        let t = fields.next().unwrap().parse::<f32>().unwrap();
        for i in 0..num_channels {
            let c = fields.next().unwrap().parse::<f32>().unwrap();
            channels[i as usize].push(c);
        }
        times.push(t);
    }

    let sample_rate = 1.0 / (times[1] - times[0]) as f64;

    AudioSamples {
        times,
        channels,
        sample_rate
    }
}
    
