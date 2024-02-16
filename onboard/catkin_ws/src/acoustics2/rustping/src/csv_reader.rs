use std::fs;
use crate::file_times::AudioSamples;

pub fn read_csv(filename: &str) -> AudioSamples {
    let file_contents = fs::read_to_string(filename).unwrap();

    let mut times = Vec::<f64>::new();
    let mut channel1 = Vec::<f64>::new();
    let mut channel2 = Vec::<f64>::new();
    let mut channel3 = Vec::<f64>::new();

    let mut lines = file_contents.lines();
    lines.next(); // skip header
    for line in lines {
        let mut fields = line.split(",");
        let t = fields.next().unwrap().parse::<f64>().unwrap();
        let c1 = fields.next().unwrap().parse::<f64>().unwrap();
        let c2 = fields.next().unwrap().parse::<f64>().unwrap();
        let c3 = fields.next().unwrap().parse::<f64>().unwrap();
        times.push(t);
        channel1.push(c1);
        channel2.push(c2);
        channel3.push(c3);
    }

    AudioSamples {
        times,
        channel1,
        channel2,
        channel3
    }
}
    
