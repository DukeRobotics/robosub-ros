use std::{io::Write, sync::Mutex};
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use std::{env, fs};

mod csv_reader;
mod binary_reader;
mod filtering;
mod file_times;
use crate::csv_reader::read_csv;
use crate::binary_reader::read_binary;
use crate::filtering::get_time_difs;

const FS: f64 = 625_000.0;
const LOWCUT: f64 = 50_000.0;
const HIGHCUT: f64 = 58_000.0;

struct PingerMeasurement {
    dx: f64,
    dy: f64,
    h1_2: f64,
    h1_3: f64,
    h2_3: f64,
}

fn get_lines(filename: &str) -> Vec<String> {
    let file = std::fs::read_to_string(filename).unwrap();
    file.lines().map(|x| x.to_string()).collect()
}

fn get_loc_measurements() -> Vec::<PingerMeasurement> {
    let start_time = std::time::Instant::now();
    // mutex of measurements
    let measurements = Mutex::new(Vec::<PingerMeasurement>::new());
    let lines = get_lines("placements.txt");

    lines.par_iter().for_each(|line| {
        println!("{}", line);
        let line = line.trim();
        let parts = line.split(" ").collect::<Vec<&str>>();
        let f = parts[0];
        let x = parts[1].parse::<f64>().unwrap();
        let y = parts[2].parse::<f64>().unwrap();
        let samples = read_csv(f);
        let (ch1_minus_ch2, ch1_minus_ch3, ch2_minus_ch3) = get_time_difs(&samples, LOWCUT, HIGHCUT, FS);
        let mut measurements_unlocked = measurements.lock().unwrap();
        for i in 0..ch1_minus_ch2.len() {
            let measurement = PingerMeasurement {
                dx: x,
                dy: y,
                h1_2: ch1_minus_ch2[i],
                h1_3: ch1_minus_ch3[i],
                h2_3: ch2_minus_ch3[i],
            };
            measurements_unlocked.push(measurement);
        }
        drop(measurements_unlocked);
        println!("Finished: {}", f)
    });

    // let zeros = vec![0.0; peak_times.len()];
    let elapsed = start_time.elapsed();
    println!("Elapsed: {:?}", elapsed);

    let mut file = std::fs::File::create("measurements.csv").unwrap();
    let header = "dx,dy,h1_2,h1_3,h2_3\n";
    file.write_all(header.as_bytes()).unwrap();
    for measurement in measurements.lock().unwrap().iter() {
        let line = format!("{},{},{},{},{}\n", measurement.dx, measurement.dy, measurement.h1_2, measurement.h1_3, measurement.h2_3);
        file.write_all(line.as_bytes()).unwrap();
    }

    measurements.into_inner().unwrap()
}

fn main() {
    // traverse folders in octants data
    let current_dir = env::current_dir().unwrap();
    // go to octants_data folder
    let current_dir = current_dir.join("octants_data");
    let start_time = std::time::Instant::now();

    let mut file = std::fs::File::create("octants.csv").unwrap();
    let header = "h1_2,h1_3,h2_3,octant\n";
    file.write_all(header.as_bytes()).unwrap();

    for entry in fs::read_dir(current_dir).unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();
        if path.is_dir() {
            let pathname = path.to_str().unwrap().to_string();
            let samples = read_binary(pathname.as_str());
            samples.iter().for_each(|s| {
                let (ch1_minus_ch2, ch1_minus_ch3, ch2_minus_ch3) = get_time_difs(s, LOWCUT, HIGHCUT, FS);
                let mmin = ch1_minus_ch2.len().min(ch1_minus_ch3.len()).min(ch2_minus_ch3.len());
                for i in 0..mmin {
                    // get innermost folder
                    let parts = pathname.split("/").collect::<Vec<&str>>();
                    let octant = parts[parts.len() - 1];
                    let line = format!("{},{},{},{}\n", ch1_minus_ch2[i], ch1_minus_ch3[i], ch2_minus_ch3[i], octant);
                    file.write_all(line.as_bytes()).unwrap();
                }
            });
            
        }
    }

    let elapsed = start_time.elapsed();
    println!("Elapsed: {:?}", elapsed);

    
}
