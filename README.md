# RoastDegreeAnalyzer
Coffee Roast Degree Analyzer using a camera without an infrared filter, a near infrared light and an enclosure.

### Hardware
- Raspberry Pi Zero 2 W
	- ![](https://assets.raspberrypi.com/static/51035ec4c2f8f630b3d26c32e90c93f1/6e7df/zero2-hero.png)
	- https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/
- Raspberry Pi Camera Module 3 NoIR
	- ![Raspberry Pi - Raspberry Pi Camera Module 3 NoIR](https://www.pi-shop.ch/media/catalog/product/cache/1/image/9df78eab33525d08d6e5fb8d27136e95/r/a/raspberry-pi-camera-module-3-noir-raspberry-pi-sc0873-40038907019459_1000x.png)
	- https://www.raspberrypi.com/products/camera-module-3/
- Waveshare Infrared LED board (850nm)
	- ![](https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/i/n/infrared-led-board_l_1_5.jpg)
	- https://www.waveshare.com/infrared-led-board.htm
- Pringles can, cardboard and ductape for enclosure. You can also use a 3D printed enclosure to make it nicer.

My research indicated that roasted coffee's spectral response is highest around the 850nm wavelength. 
![](https://content.instructables.com/F7A/FR8S/JOONL4U7/F7AFR8SJOONL4U7.png?auto=webp&frame=1&fit=bounds&md=MjAxOC0xMS0xOSAxNzowMToyNi4w)
https://www.instructables.com/Degree-of-Roast-Infrared-Analyzer-for-Coffee-Roast/

A Pringles can is used to create an enclosure to ensure that no light enters the measurement area, other than the 850nm wavelength light from the LED.

The Raspberry Pi Camera Module 3 NoIR is essential as it doesn't have an infrared filter to cut off that 850nm wavelength light and it also still has a fairly good spectral response in that range. Other camera modules don't seem to have such a good spectral response in that range.
https://www.researchgate.net/publication/386554881/figure/fig6/AS:11431281295727983@1733714933410/Spectral-Response-of-Raspberry-Pi-Camera-Module-3-NoIR-Response-of-the-Red-R-Green.ppm

Important Note:
Do not power the infrared LED off of the Raspberry Pi. It can damage the RPi like that. Run it off of a separate USB charger by cutting up a Micro USB cable and connecting the exposed wires to a 5v -> 3.3v step down DC voltage converter. Then the 3.3v output of that can be used to power the LED.

## How it works:
1. The server captures a raw image from the camera
2. Compresses the image and sends it over to the client
3. The client analyses the target area in the image, calculates min/max/avg/mean values
4. The client calculates the Agtron scale value using the average value and reports it on screen

### Agtron Scale Value calculation
The Agtron scale value is calculated using a cubic function. It has been calibrated against the Crc-80 reference cards by: taking multiple captures, taking the average and fitting a cubic function over the values.
![](https://cmsale.com/files/thumbs/products/CRC-80/crc80.png/900_900_crop.png?ts=1628518933&pn=product_big)
Then this cubic function is used to calculate the Agtron (Gourmet) scale (0-100) Value given an input camera reading. It is designed to return 0 for carbonized coffee and 100 for under-developed.
While it is possible to just compare coffee grounds to the reference cards, according to the Agtron M-Basic II manual the visual resolution of our eyes with medium roasts is only around +-6 Agtron points.
https://cmsale.com/products/analysis/color-analyzers/coffee-roast-color-crc-80

## Coffee bed preparation
The amount of light reflected from a perfectly flat surface is significantly greater than that of a rough one. Therefore the readings a color meter delivers with fine grounds are higher than on coarser grounds of the same coffee.
Only freshly roasted, room temperature coffee must be used as decomposition can lower a samples color measurement result by up to 3 points on the Agtron scale in 24 hours. Darker roast change even faster over time.
Coffee must be ground to about espresso and then sifted with a Kruve Sifter to between 300-500um
https://www.kruveinc.com/pages/kruve-sifter

The coffee is then placed in the Pringes can plastic cap.  It needs to be evenly distributed with a WDT tool and tamped to achieve a uniform and level coffee bed. 
This method of preparation ensures that we are able to take consistent readings and achieve good mapping to the Agtron scale.
https://artisan-roasterscope.blogspot.com/2023/03/understanding-roast-color.html

### Measurement
Wait a little bit of time so that the LED light can warm up and its output becomes stable.
Then after preparing the coffee bed and adding the enclosure open the client.
Adjust the target area circle so that it covers the whole coffee bed. Make sure it doesn't touch the enclosure.
Take multiple measurements (eg. 5), and between them use the WDT tool to agitate the coffee bed and re-tamp. This will ensure that measurement errors are minimised. 


## Software Dependencies
Install dependencies with:
`sudo apt install git cmake libssl-dev libcamera-dev`

## Building and running
Build using cmake:
`mkdir build && cd build && cmake .. && make`

Run server:
`./RoastDegreeAnalyzer`
 
 Then open Client.html on your client device and connect to the Raspberry Pi's IP address

