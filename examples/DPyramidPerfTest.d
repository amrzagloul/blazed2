module pyramidPerfTest;

import std.datetime;
import std.stdio;

import blaze.all;

void main()
{
    auto test = new DPyramidPerfTest();
}

class DPyramidPerfTest
{
    bzShapeDef sd;
    bzBodyDef  bd;
    bzBody  rBody;
    bzWorld world;
    bzAABB  worldAABB;

    this()
    {
        StopWatch timer;
        TickDuration dt;
        float timeStep         = 1.0f / 60.0f;
        int velocityIterations = 10;
        int positionIterations = 10;

        createWorld();
        createTest();
        timer.start();

        foreach (i; 0 .. 1000)
        {
            timer.reset();
            world.step(timeStep, velocityIterations, positionIterations);
            dt += timer.peek();
        }

        writefln("%s ms.", dt.msecs);
    }

    void createWorld()
    {
        worldAABB.lowerBound.set(-200.0f, -100.0f);
        worldAABB.upperBound.set(200.0f, 200.0f);
        auto gravity = bzVec2(0.0f, -10.0f);
        auto doSleep = true;
        world = new bzWorld(worldAABB, gravity, doSleep);
    }

    void createTest()
    {
        sd = new bzPolyDef();
        sd.setAsBox(50.0f, 10.0f);

        bzVec2 position = bzVec2(0.0f, -10.0f);
        float  angle    = 0.0f;
        auto bd         = new bzBodyDef(position, angle);
        bzBody ground   = world.createBody(bd);
        ground.createShape(sd);

        sd = new bzPolyDef();
        float a = 0.5f;
        sd.setAsBox(a, a);
        sd.density = 5.0f;

        bzVec2 x = bzVec2(-10.0f, 1.0f);
        bzVec2 y;
        bzVec2 deltaX = bzVec2(0.5625f, 2.0f);
        bzVec2 deltaY = bzVec2(1.125f, 0.0f);

        foreach (i; 0 .. 25)
        {
            y = x;

            foreach (j; i .. 25)
            {
                position = y;
                bd       = new bzBodyDef(position, angle);
                sd       = new bzPolyDef();
                a        = 0.5f;
                sd.setAsBox(a, a);
                sd.density = 5.0f;
                rBody      = world.createBody(bd);
                rBody.createShape(sd);
                rBody.setMassFromShapes();
                y += deltaY;
            }

            x += deltaX;
        }
    }
}
