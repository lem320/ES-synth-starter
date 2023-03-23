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
  Theoretical minimum initiation interval: <br />
  Measured maximum execution time: <br />

## Envelope
  ### Implementation
  ### Timings
  Theoretical minimum initiation interval: <br />
  Measured maximum execution time: <br />

## Waveform Selection
  ### Implementation
  Square wave: The output is changed to make it the most significant bit of the saw-tooth wave.<br />

  Triangle wave: The phase accumulator is increased until one more step would cause it to overflow, then it switches states to decrease. It repeats this action until the value would overflow the other way, and continues to repeat. <br />
  ### Timings
  Theoretical minimum initiation interval: <br />
  Measured maximum execution time: <br />

## Low Frequency Oscillator
  ### Implementation
  ### Timings
  Theoretical minimum initiation interval: <br />
  Measured maximum execution time: <br />



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
    
 
