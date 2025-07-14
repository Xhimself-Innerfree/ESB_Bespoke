# ESB Demo by Flavin Neuromachines Lab

[Flavin Neuromachines Lab](https://flavinlab.io/)

This is a demo to show the possibility of that a Node can work as a PRX in its star topology and work as a PTX in another star topology to show it's scalable.

## How to set it up

the Xesb_prx and Xesb_ptx are ESB PRX and ESB PTX firmware. Flashed with this, it can only work as a PRX or a PTX. To send a packet, open the COM of the DK flashed with PTX, press RESET button on your DK. Some LOGs will be printed on the Command Line Interface (CLI). Then, every time a "1" is pressed on the keyboard, the LED on PTX will be toggled, and the PTX will send a packet, the PRX receives the packet and print the content on the CLI, the LED on PRX will be toggled. To be mentioned, there will be a time gap between the time two led toggled. Because the packet is send only once in one second controlled by a timer. Plus, this is not the max rate of ESB, I design it to work like this. Want to try the max datarate? Use the esb_prx and esb_ptx. set the line 234 K_MSEC() in main.c of esb_ptx. It's the time gap between two packets.

The ESB_merged is a firmware that stays on PRX state for the most of time. While a "1" is read from COM, it will switch to PTX. After sending the packets, it will switch to PRX automatically. The ESB PRX and PTX find each other by address pair. If the address is the same on two ends, they can communicate. So by setting the address pair correctly, a PRX in a star topology could be the PTX in another star topology. In this way, the Half-Duplex ESB could feel full-duplex.
