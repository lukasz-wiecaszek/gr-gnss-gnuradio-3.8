# gr-gnss

This is my attempt to build GPS (or more generally Global Navigation Satelite Systems)
receiver with a little help of gnuradio.
The best way (at least to me) to learn a new technology is to try to build a project
making use of that technology.
This is the main reason and motivation why this project was created.
At this stage of development only acquisition and tracking block is prepeared.
If interested, please see especially gps_cross_correlation_v5.grc
where you will be able to spot navigation message bits on the GUI Time Sink.

## How to build this module

First of all you will need all the required dependencies.
Most likely you have them already installed.
I am assuming that you already have gnuradio installed. If not, please do so first.

You can go to [Installing GR from Binaries](https://wiki.gnuradio.org/index.php/InstallingGR#From_Binaries)
or [Installing GR from Source](https://wiki.gnuradio.org/index.php/InstallingGR#From_Source)
depending on your needs.

By the way, I am using gnuradio 3.8.

Then, as with every gnuradio module we use cmake.

```
  $ mkdir build
  $ cd build
  $ cmake ..
  $ make
  $ sudo make install
  $ sudo ldconfig
```

## What can I do with this module

Currently not so much. As already mentioned, this is my way of learning new technology.
What you can do is to look into examples and see how this spreading code correlation works.
Finally gps_cross_correlation_v5.grc shows decoded navigation message bits in the GUI Time Sink.
