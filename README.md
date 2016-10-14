# Introduction

This is the GNU Radio OOT module for parsing and displaying of Flex FFT data.  It is a modified version of the GNURadio QT Time Sink that only displays Flex FFT data.  The original source can be found in the GNURadio project under the gr-qtgui module:

https://github.com/gnuradio/gnuradio

# Installation

If you already have GNURadio installed, you can install this block through the standard install process.

	git clone https://github.com/drs-ss/gr-flexfft
	cd gr-flexfft
	mkdir build
	cd build
	cmake ..
	make
	sudo make install
	sudo ldconfig

## Using the block

The gr-flexfft block can be used in two ways.  It can be fed data from a UDP Source that has Flex FFT packets (select 'Byte' as input type) or it can be hooked up to a Polaris Source block (select 'Float' as input type).

The block will automatically adjust it's X axis to match what the context packet information contains.  Units displayed are MHz on the X axis and dBm on the Y axis.
