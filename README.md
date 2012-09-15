sign-detector
=============

Detects signs near the center of an image and returns the coordinates of the corners in (top-left, top-right, bottom-right, bottom-left) order.

To compile, run the following command:

    $ g++ src/* -o build/sign_detector -lopencv_core -lopencv_highgui -lopencv_imgproc

Then, to use, run the executable with an image filename as an argument. For example:

    $ build/sign_detector stills/000006.png
