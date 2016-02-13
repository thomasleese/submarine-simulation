package org.suhps.simulation;

import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.*;
import com.badlogic.gdx.utils.Disposable;

public class Submarine implements Disposable {

    private float mWidth;
    private float mCrossSectionalArea;
    private float mDragCoefficient;
    private float mLiftCoefficientSlope;
    private float mSpinningDragCoefficient;
    private float mFinsCrossSectionalArea;
    private float mFinsLiftCoefficientSlope;
    private float mFinsDragCoefficient;

    private Body mBody;

    public Submarine(float width, float height, float mass, float crossSectionalArea,
                     float dragCoefficient, float liftCoefficientSlope, float spinningDragCoefficient,
                     float finsCrossSectionalArea, float finsLiftCoefficientSlope, float finsDragCoefficient,
                     float initialX, float initialY, float initialSpeed, float initialAngle, World world) {
        mWidth = width;

        mCrossSectionalArea = crossSectionalArea;
        mDragCoefficient = dragCoefficient;
        mLiftCoefficientSlope = liftCoefficientSlope;
        mSpinningDragCoefficient = spinningDragCoefficient;
        mFinsCrossSectionalArea = finsCrossSectionalArea;
        mFinsLiftCoefficientSlope = finsLiftCoefficientSlope;
        mFinsDragCoefficient = finsDragCoefficient;

        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyDef.BodyType.DynamicBody;
        bodyDef.position.set(initialX, initialY);
        bodyDef.angle = initialAngle;

        mBody = world.createBody(bodyDef);

        PolygonShape shape = new PolygonShape();
        shape.setAsBox(width / 2f, height / 2f);

        float area = width * height;
        float density = mass / area;

        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = shape;
        fixtureDef.density = density;
        fixtureDef.friction = 0f;
        fixtureDef.restitution = 0f;

        mBody.createFixture(fixtureDef);

        shape.dispose();

        mBody.setLinearVelocity(initialSpeed, 0f);
    }

    @Override
    public void dispose() {

    }

    private void drawForce(ShapeRenderer renderer, Vector2 position, Vector2 value) {
        Vector2 end = Vector2.X.set(value).scl(0.02f).add(position);
        renderer.x(position, 0.1f);
        renderer.line(position, end);
    }

    private float wrapAngle(float angle) {
        while (angle > MathUtils.PI) {
            angle -= MathUtils.PI2;
        }

        while (angle < -MathUtils.PI) {
            angle += MathUtils.PI2;
        }

        return angle;
    }

    private void applyThrust(ShapeRenderer renderer, float thrust, float theta) {
        Vector2 thrustVector = new Vector2(thrust, 0);
        thrustVector.rotate(theta);
        thrustVector.rotateRad(mBody.getAngle());

        Vector2 position = mBody.getWorldPoint(new Vector2(-mWidth / 2f, 0f));

        mBody.applyForce(thrustVector, position, true);

        renderer.setColor(1f, 0f, 0f, 1f);
        drawForce(renderer, position, thrustVector);
    }

    private void applyDrag(ShapeRenderer renderer, float fluidDensity) {
        Vector2 velocity = mBody.getLinearVelocity();

        float v2 = velocity.len() * velocity.len();
        float value = 0.5f * fluidDensity * mCrossSectionalArea * mDragCoefficient * v2;
        Vector2 drag = velocity.cpy().nor().scl(-value);

        mBody.applyForceToCenter(drag, true);

        Vector2 position = mBody.getWorldCenter().cpy();

        renderer.setColor(0f, 1f, 0f, 1f);
        drawForce(renderer, position, drag);
    }

    private void applyLift(ShapeRenderer renderer, float fluidDensity) {
        Vector2 velocity = mBody.getLinearVelocity();
        float angle = wrapAngle(mBody.getAngle());

        float alpha = wrapAngle(angle - wrapAngle(velocity.angleRad()));

        if (Math.abs(alpha) < MathUtils.degreesToRadians * 15) {
            float liftCoefficient = alpha * mLiftCoefficientSlope;

            float v2 = velocity.len() * velocity.len();

            float value = 0.5f * fluidDensity * mCrossSectionalArea * liftCoefficient * v2;

            Vector2 lift = velocity.cpy().nor().rotate90(1).scl(value);

            Vector2 position = mBody.getWorldPoint(new Vector2(mWidth / 4f, 0f));

            mBody.applyForce(lift, position, true);

            renderer.setColor(0f, 0f, 1f, 1f);
            drawForce(renderer, position, lift);
        }
    }

    private void applyFinsLift(ShapeRenderer renderer, float fluidDensity) {
        Vector2 velocity = mBody.getLinearVelocity();
        float angle = wrapAngle(mBody.getAngle());

        float alpha = wrapAngle(angle - wrapAngle(velocity.angleRad()));

        if (Math.abs(alpha) < MathUtils.degreesToRadians * 15) {
            float liftCoefficient = alpha * mFinsLiftCoefficientSlope;

            float v2 = velocity.len() * velocity.len();

            float value = 0.5f * fluidDensity * mFinsCrossSectionalArea * liftCoefficient * v2;

            Vector2 lift = velocity.cpy().nor().rotate90(1).scl(value);

            Vector2 position = mBody.getWorldPoint(new Vector2(-mWidth / 2f, 0f));

            mBody.applyForce(lift, position, true);

            renderer.setColor(1f, 0.5f, 0.3f, 1f);
            drawForce(renderer, position, lift);
        }
    }

    private void applyFinsDrag(ShapeRenderer renderer, float fluidDensity) {
        Vector2 velocity = mBody.getLinearVelocity();

        float v2 = velocity.len() * velocity.len();
        float value = 0.5f * fluidDensity * mFinsCrossSectionalArea * mFinsDragCoefficient * v2;
        Vector2 drag = velocity.cpy().nor().scl(-value);

        mBody.applyForceToCenter(drag, true);

        Vector2 position = mBody.getWorldCenter().cpy();

        renderer.setColor(0.9f, 0f, 0.7f, 1f);
        drawForce(renderer, position, drag);
    }

    private void applySpinningDrag(float fluidDensity) {
        float angularVelocity = mBody.getAngularVelocity();

        float v2 = angularVelocity * angularVelocity;
        float value = (0.5f * fluidDensity * mCrossSectionalArea * mSpinningDragCoefficient * v2) / mWidth;

        float drag = -value;
        if (angularVelocity < 0) {
            drag = value;
        }

        mBody.applyTorque(drag, true);
    }

    public void update(ShapeRenderer renderer, float thrust, float theta, float fluidDensity) {
        applyThrust(renderer, thrust, theta);
        applyDrag(renderer, fluidDensity);
        applyLift(renderer, fluidDensity);
        applyFinsLift(renderer, fluidDensity);
        applyFinsDrag(renderer, fluidDensity);
        applySpinningDrag(fluidDensity);
    }

    public Vector2 getWorldCenter() {
        return mBody.getWorldCenter();
    }

    public float getAngle() {
        return mBody.getAngle();
    }

}
