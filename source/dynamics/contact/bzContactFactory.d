/*
 * Copyright (c) 2007-2008, Michael Baczynski
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the polygonal nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
module blaze.dynamics.contact.bzContactFactory;

import  blaze.dynamics.bzBody,
        blaze.dynamics.contact.generator.bzCircleContact,
        blaze.dynamics.contact.generator.bzCircleFluidContact,
        blaze.dynamics.contact.generator.bzPolyCircleContact,
        blaze.dynamics.contact.generator.bzPolyContact,
        blaze.dynamics.contact.generator.bzPolyFluidContact,
        blaze.dynamics.contact.generator.bzEdgeCircleContact,
        blaze.dynamics.contact.generator.bzPolyEdgeContact,
        blaze.collision.shapes.bzShapeType,
        blaze.collision.shapes.bzShape,
        blaze.collision.bzCollision,
        blaze.dynamics.contact.bzContact;

alias bzContact function(bzShape, bzShape) ContactCreateFcn;

/** bzContact register */
struct bzContactRegister {
    ContactCreateFcn createFcn;
    bool primary;
}

class bzContactFactory {

    this() {
        initializeRegisters();
    }

    ///
    void initializeRegisters() {
        addType(&bzCircleContact.create, bzShapeType.CIRCLE, bzShapeType.CIRCLE);
        addType(&bzCircleFluidContact.create, bzShapeType.CIRCLE, bzShapeType.FLUID);
        addType(&bzEdgeCircleContact.create, bzShapeType.EDGE, bzShapeType.CIRCLE);
        addType(&bzPolyCircleContact.create, bzShapeType.POLYGON, bzShapeType.CIRCLE);
        addType(&bzPolyContact.create, bzShapeType.POLYGON, bzShapeType.POLYGON);
        addType(&bzPolyEdgeContact.create, bzShapeType.POLYGON, bzShapeType.EDGE);
        addType(&bzPolyFluidContact.create, bzShapeType.POLYGON, bzShapeType.FLUID);
        s_initialized = true;
    }

    ///
    void addType(ContactCreateFcn createFcn, int type1, int type2) {

        assert(bzShapeType.UNKNOWN < type1 && type1 < bzShapeType.SHAPE_COUNT);
        assert(bzShapeType.UNKNOWN < type2 && type2 < bzShapeType.SHAPE_COUNT);

        s_registers[type1][type2].createFcn = createFcn;
        s_registers[type1][type2].primary = true;

        if (type1 != type2) {
            s_registers[type2][type1].createFcn = createFcn;
            s_registers[type2][type1].primary = false;
        }
    }

    ///
    bzContact create(bzShape shape1, bzShape shape2) {

        if (!s_initialized) {
            initializeRegisters();
            s_initialized = true;
        }

        bzShapeType type1 = shape1.type;
        bzShapeType type2 = shape2.type;

        assert(bzShapeType.UNKNOWN < type1 && type1 < bzShapeType.SHAPE_COUNT);
        assert(bzShapeType.UNKNOWN < type2 && type2 < bzShapeType.SHAPE_COUNT);

        ContactCreateFcn createFcn = s_registers[type1][type2].createFcn;
        if (createFcn) {
            if (s_registers[type1][type2].primary) {
                return createFcn(shape1, shape2);
            } else {
                bzContact c = createFcn(shape2, shape1);
                for (int i = 0; i < c.manifoldCount; ++i) {
                    c.manifold.normal *= -1;
                }
                return c;
            }
        } else {
            return null;
        }
    }

    void destroy(bzContact contact) {
        assert(s_initialized);

        if (contact.manifoldCount > 0)
        {
            contact.shape1.rBody.wakeup();
            contact.shape2.rBody.wakeup();
        }

        bzShapeType type1 = contact.shape1.type;
        bzShapeType type2 = contact.shape2.type;

        assert(bzShapeType.UNKNOWN < type1 && type1 < bzShapeType.SHAPE_COUNT);
        assert(bzShapeType.UNKNOWN < type2 && type2 < bzShapeType.SHAPE_COUNT);

        delete contact;
    }

private:

    bzContactRegister s_registers[bzShapeType.SHAPE_COUNT][bzShapeType.SHAPE_COUNT];
    bool s_initialized;

}
