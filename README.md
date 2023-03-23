# ES-synth
<!-- 
# Contents
- [Tasks](#tasks)
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
- [Dependencies](#dependencies) -->

# Features
## Polyphony
  To implement this, the code had to be vastly changed into an OOP style, having the keyboard as a class named Octave, and each key being a class named Notes. The Octave class stored a Note class for each key on the keyboard. Inside the Note class, each key has its own phaseAcc variable, which is changed upon a press of the key the Note class refers to. The octave class then takes the average of all of the phaseAccs from the Note classes to output a superimposed wave of all the keys being played.
 

## Envelope
  This keyboard has an option to add an envelope onto the output. This is set in the menu on the Master Keyboard's dislpay.
  
  The envelope is implemented by multiplying the variable phaseAcc (the output sound of a note) by a variable that has a changing amplitude. This amplitude is between 0 and 1 to either play the note at normal volume or make it quieter. <br />
  
  The envelope was split up into 4 states:
  * Initial rise (0): This occurs when a key is initially pressed. It quickly increases the envelope amplitude until it reaches 1, when it then moves to state 1.
  * First fall (1): This occurs after the note reaches its peak. It then decreases envelope amplitude slowly until it reaches 0.5, so that the note is being played at half its normal volume. Once the envelope amplitude reaches 0.5, it moves into state 2.
  * Constant amplutude (2): This holds the note at a constant volume, in this case at half it's peak. It does this until the key is released, which is when the envelope moves into state 3.
  * Final fall (3): This slowly decreases the envelope amplitde to 0. Once it reaches 0, the envelope moves back into state 0, completeing the cycle. <br />

  The enevelope was implemented in the Note class so that each key has it's own envelope. <br />
  

## Waveform Selection
  The wave chosen to be used is set by the third knob and the type of wave to be outputted is stored in the Note class. This is because the calculation for the phase accumulator is done here.
  
  * Sawtooth: The phase acculmulator is incremented by a step size, which is determined by the frequency of the key, with every cycle. The phase accumulator overflows when it passes the maximum value it can store, which leads to it resetting (in reailty it does not necessarily go back to 0) and then it begins its increase again.
  * Square wave: The output is changed to make it the most significant bit of the saw-tooth wave.<br />
  * Sine wave: A look up table is made in the setup function of the code. It stores a sine wave of frequency 1Hz, with 5000 samples. A time variable is then stores in the Note class. This variable is the time since the key was first pressed modulo the period of the wave the note is outputting. This is to prevent the time variable increasing to a large number and overflowing. The time it then used to index the LUT and this value is stored in the phase accumulator.
  * Triangle wave: The phase accumulator is increased until one more step would cause it to overflow, then it switches states to decrease. It repeats this action until the value would overflow the other way, and continues to repeat. <br />


## Filtering
  This keyboard has an option to apply a Low Pass Filter onto the output. This is set in the menu on the Master Keyboard's dislpay. The filter has a cut off frequency of 1kHz to not attenuate the desired frequencies being played from the keyboard.
  
  The LPF was implemented using the Filter class that stores the coefficients for each term as well as previous values for the inputted and outputted values.


## Low Frequency Oscillator
  This keyboard has an option to add a Low Frequency Oscillator onto the output. This is set in the menu on the Master Keyboard's dislpay.
  
  The LFO was implemented by superimposing a low frequency and small amplitude sine wave onto the output of each key in the Note class.

  
  
 ## Advanced Display
  A user friendly user interface was implemented on the display which shows a menu on the left, listing all additional features that can be turned on or off, and the current volume and waveform being outputted. As well as this, the current keys that are being played are displayed allong the bottom of the display.
  
  The UI is interfaces with the knobs. Each knob has a different role:
  * Knob 1: Scroll through the additional features
  * Knob 2: Changed the status of the feature selected by Knob 1
  * Knob 3: Change the waveform
  * Knob 4: Change the volume
 

# Tasks
|Priority     | Task              | Type       | Initiation Interval | Execution Time  | Characterisation |
|-------------|-------------------|------------|---------------------|-----------------|------------------|
|1            | scanKeysTask      | Thread     | 20 ms               | 84 µs           | Sets value of keyArray and sets the notes being played |  
|2            | decodeTask        | Thread     | 25.2 ms             | 0.03 µs         | Stores CAN message received                            | 
|3            | CAN_TX_Task       | Thread     | 60 ms               | 12 µs           | Sends CAN message                                      |
|4            | displayUpdateTask | Thread     | 100 ms              | 15.2 ms         | Creates and sends data buffer to display               |
|      -      | sampleISR         | Interrupt  | 45.5 µs             | 26 µs           | Sets the analogue voltage to the speaker               |
|      -      | CAN_RX_ISR        | Interrupt  | 700 µs              | 3 µs            | Receives CAN message                                   | 
|      -      | CAN_TX_ISR        | Interrupt  | 700 µs              | 3 µs            | Gives MUTEX to CAN_TX_Task                             | 



# Rate Monotonic Scheduler
A critical instant analysis of the rate monotonic scheduler, showing that all deadlines are met
under worst-case conditions

# CPU Utilisation
A quantification of total CPU utilisation

# Synchronisation

## Shared data structures
* Octave: This is shared between scanKeys (to set the key being pressed), displayUpdateTask (to display the keys being pressed) and sampleISR (to retrieve the sound for the speaker to make).
* Note: This is shared between scanKeys (called from Octave to set if the key it corresponds to is being pressed) and sampleISR (to retrieve the sound from each note to be summed by an Octave class function).
* DisplayItem: This is shared between scanKeys (to get the level to send over CAN) and displayUpdateTask (to set the level).
* Knob: This is shared between scanKeys (to get the level to send over CAN), displayUpdateTask (to display the level) and sampleISR (to set the volume using the forth knob).

Safe access in these classes is guaranteed by using atomic stores to store variables within the class that are shared between the tasks.

## Shared vectors
* keyArray: This is shared between scanKeys and displayUpdateTask. 
* CAN_TX_Semaphore: This is shared between CAN_TX_ISR and CAN_TX_Task

These cannot be stored atomically as they are vectors, so MUTEXs are used so that only one task can read/write to the variables at any time.

# Dependencies
An analysis of inter-task blocking dependencies that shows any possibility of deadlock

## Blocking functions used include:
* vTaskDelayUntil: This function blocks a task from running until a set time has elapsed. This function cannot cause a deadlock though as it can only block for a finite amount of time.
* xSemaphoreTake: This function blocks a task as it waits to be able to take a MUTEX once the task that has the MUTEX has completed its goal, it will call xSemaphoreGive so that another task can take the MUTEX if it takes it in time. This function can cause a deadlock if a task holds the MUTEX indefinitely as it will cause all other functions waiting for the MUTEX to wait indefinitely.

Only the above blocking functions are used and the xSemaphoreTake function is used carefully to prevent a deadlock from happening. This is done by having each task always release the MUTEX, meaning the MUTEX is never infinitely held.
