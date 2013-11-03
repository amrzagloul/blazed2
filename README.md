BlazeD2
=======

BlazeD2 is a D version 2 port of Blaze, a 2D game physics engine based on Box2D.
It's a direct port with no functional changes. Only the DSSS build system was replaced
by a Visual D project.

Alternative
===========

BlazeD2 is lagging behind today's box2d. Check out Dchip instead: 
https://github.com/AndrejMitrovic/dchip


Original
========

The original Blaze can be found on dsource.org:
http://www.dsource.org/projects/blaze

Examples
========

There are several examples in the `examples` directory.
To test hello world run:

```
rdmd -I../source helloWorld.d
```

Limitations
===========

The large test-bed example in the D1 Blaze project is based on Tango and
team0xf's OMG, I did NOT port it. Consequently the best showcase for
Blaze capabilities still is on the original dsource project page.

