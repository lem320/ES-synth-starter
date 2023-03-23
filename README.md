# ES-synth-starter

  Use this project as the starting point for your Embedded Systems labs and coursework.
  
  [Lab Part 1](doc/LabPart1.md)
  
  [Lab Part 2](doc/LabPart2.md)

# Contents
- [Features](#features)
  * [Polyphony](#polyphony)
    + [Implementation](#implementation)
    + [Timings](#timings)
  * [Envelope](#envelope)
    + [Implementation](#implementation-1)
    + [Timings](#timings-1)
  * [Waveform Selection](#waveform-selection)
    + [Implementation](#implementation-2)
    + [Timings](#timings-2)
  * [Low Frequency Oscillator](#low-frequency-oscillator)
    + [Implementation](#implementation-3)
    + [Timings](#timings-3)
- [Rate Monotonic Scheduler](#rate-monotonic-scheduler)
- [CPU Utilisation](#cpu-utilisation)
- [Synchronisation](#synchronisation)
- [Dependencies](#dependencies)

# Features
## Polyphony
  ### Implementation
  An identification of all the tasks that are performed by the system with their method of implementation, thread or interrupt
  
  To implement this, the code had to be vastly changed into an OOP style, having the keyboard as a class named Octave, and each key being a class named Notes. The Octave class stored a Note class for each key on the keyboard. Inside the Note class, each key has its own phaseAcc variable, which is changed upon a press of the key the Note class refers to. The octave class then takes the average of all of the phaseAccs from the Note classes to output a superimposed wave of all the keys being played.
  ### Timings
  Theoretical minimum initiation interval: <br />
  Measured maximum execution time: <br />

## Envelope
  ### Implementation
  The envelope is implemented by multiplying the variable phaseAcc (the output sound of a note) by a variable that has a changing amplitude. This amplitude is between 0 and 1 to either play the note at normal volume or make it quieter. <br />
  
  The envelope was split up into 4 states:
  * Initial rise (0): This occurs when a key is initially pressed. It quickly increases the envelope amplitude until it reaches 1, when it then moves to state 1.
  * First fall (1): This occurs after the note reaches its peak. It then decreases envelope amplitude slowly until it reaches 0.5, so that the note is being played at half its normal volume. Once the envelope amplitude reaches 0.5, it moves into state 2.
  * Constant amplutude (2): This holds the note at a constant volume, in this case at half it's peak. It does this until the key is released, which is when the envelope moves into state 3.
  * Final fall (3): This slowly decreases the envelope amplitde to 0. Once it reaches 0, the envelope moves back into state 0, completeing the cycle. <br />

  The enevelope was implemented in the Note class so that each key has it's own envelope. <br />
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
    
 
