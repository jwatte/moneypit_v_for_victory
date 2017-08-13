# Public Moneypit V for Victory source code

This repository contains the source code that ran the autonomous rover 
Moneypit V for Victory at the autonomous car race meet-up on August 12, 2017.
The rover placed second.

The vision code runs on a Raspberry Pi 2, and lives inside the `camcam` 
directory. There is also a helper tool called `showip` which looks at the 
state of network interfaces, and every so often, tries to cycle them if they 
are not connected, as well as prints the IP address of the RPi on the display.

Talking to the roboclaw controllers and Dynamixel servos is done by a 
Teensy 3.2 microcontroller. The code for that lives in `mpv_teensy`.

Finally, this code uses some helpers to read/write images from the "stb" 
public repository (original source github.com/nothings/stb) which is included 
for convenience.

The Teensy code is built using `Arduino for Raspberry Pi version 1.8.3`, with 
the `Teensyduino` libraries installed from `pjrc.com`.
The Raspberry Pi code is built using standard `C++` and `make`.
As is so often the case, this code is a living beast, mainly intended to do 
the one thing, with no real concern for discoverability or maintainability. 

## Some modules

I release it because there are some useful bits that could be re-purposed:

  - The slimmed-down `RaspiVid.c` code, which can serve as a basis of other 
  code that wants to read the RPi camera. It's less verbose and contains less 
  special cases that you won't need than the original from Broadcom.

  - The `yuv` code provides a convenient converter between YUV420 and RGB 
  data.

  - The `settings` module is simple yet highly useful for text-mode settings 
  files.

  - The `pipeline` and `queue` modules provide a simple way of sending data 
  between multiple worker threads in a somewhat organized fashion.

  - The `detect_inner` and `project` modules show how to do feature extraction 
  for the yellow line on the track, and re-project it into a flat space that 
  is then clustered. It's perhaps a useful illustration, but not intended to 
  be highly reusable.

  - The `widget` stucc in the `gui` module should be kept away from children 
  and human beings with good taste, lest it rot out your eyes and gives you 
  nightmares.

## Training data

In the `training_data` directory, there are images collected while running 
the track; there is also a "training.csv" file that contains tuples of input
filename, output steering, and output throttle response. This might be useful 
for someone who wants to train a neural network to clone the behavior of my 
rover :-)

## Utilities

There are a few utilities:

  - `mkpng` turns a raw RGB or YUV file into a `.png` file you can view with 
  tools such as `feh` or whatever. Say `./mkpng {input.yuv} yuv` to create an 
  output file called raw.png; default size is 320x240.

  - `mkdetect` runs the detction algorithm, given an input 320x240 image, and 
  prints out what the algorithm thinks the throttle/steer response should be.

## License

I place all this code in the public domain. I claim no responsibility for the 
correctness of the code -- in fact, it does have bugs, and should not be used 
in any kinds of systems where money, lives, animals, or anything else you 
care about are at stake. You take full responsibility for whatever you manage 
to do (or, more likely, fail to do) using this code.

2017 Jon Watte
