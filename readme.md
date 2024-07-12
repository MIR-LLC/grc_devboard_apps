A simple application demonstrating how a voice-controlled relay operates. 

When the word “Robot” is spoken, the relay activates and sends a signal to a specific pin for 10 seconds. It can be stopped prematurely using the “Stop” command.


# How to build
Specify target device (AI_MODULE or GRC_DEVBOARD) in BUILD_TARGET variable:
```bash
idf.py -DBUILD_TARGET=<target device> build
idf.py -p <port> flash monitor
```
