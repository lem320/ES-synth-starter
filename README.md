# ES-synth-starter

  Use this project as the starting point for your Embedded Systems labs and coursework.
  
  [Lab Part 1](doc/LabPart1.md)
  
  [Lab Part 2](doc/LabPart2.md)

## Additional Information
  [Handshaking and auto-detection](doc/handshaking.md)

# Features
## Polyphony
  ### Implementation
  An identification of all the tasks that are performed by the system with their method of implementation, thread or interrupt
  ### Timings
  A characterisation of each task with its theoretical minimum initiation interval and measured
maximum execution time

## Envelope
  ### Implementation
  ### Timings

## Waveform Selection
  ### Implementation
  Square wave: Implemented through making the output the most significant bit of the saw tooth wave.
  Triangle wave: Implemented through increasing the phase accumulator until it is about to overflow if another step is done - then it switches states to decrease its value until before it overflows the opposite way. 
  ### Timings

## Low Frequency Oscillator
  ### Implementation
  ### Timings



# Rate Monotonic Scheduler
A critical instant analysis of the rate monotonic scheduler, showing that all deadlines are met
under worst-case conditions

# CPU Utilisation
A quantification of total CPU utilisation

# Synchronisation
An identification of all the shared data structures and the methods used to guarantee safe access
and synchronisation

# Dependencies
An analysis of inter-task blocking dependencies that shows any possibility of deadlock
    
 
