package org.suhps.simulation;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.ControllerListener;
import com.badlogic.gdx.controllers.PovDirection;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.box2d.*;
import com.badlogic.gdx.utils.Disposable;

public class Submarine implements Disposable, InputProcessor, ControllerListener {

    private static final String TAG = "Submarine";

    private static final float MAX_THETA = 20f;
    private static final float MAX_THRUST = 150f;

    private float mWidth;
    private float mCrossSectionalArea;
    private float mDragCoefficient;
    private float mLiftCoefficientSlope;
    private float mSpinningDragCoefficient;
    private float mFinsCrossSectionalArea;
    private float mFinsLiftCoefficientSlope;
    private float mFinsDragCoefficient;

    private Body mBody;

    private float mThrust = 0;
    private float mTheta = 0;

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

    private void applyThrust(ShapeRenderer renderer) {
        Vector2 thrustVector = new Vector2(mThrust, 0);
        thrustVector.rotate(mTheta);
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

    public void update(ShapeRenderer renderer, float fluidDensity) {
        if (Gdx.input.isKeyPressed(Input.Keys.LEFT)) {
            mTheta += 1f;
        }

        if (Gdx.input.isKeyPressed(Input.Keys.RIGHT)) {
            mTheta -= 1f;
        }

        if (Gdx.input.isKeyPressed(Input.Keys.UP)) {
            mThrust -= 1f;
        }

        if (Gdx.input.isKeyPressed(Input.Keys.DOWN)) {
            mThrust += 1f;
        }

        mTheta = MathUtils.clamp(mTheta, -MAX_THETA, +MAX_THETA);
        mThrust = MathUtils.clamp(mThrust, 0, MAX_THRUST);
        applyThrust(renderer);
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

    @Override
    public void connected(Controller controller) {

    }

    @Override
    public void disconnected(Controller controller) {

    }

    @Override
    public boolean buttonDown(Controller controller, int buttonCode) {
        return false;
    }

    @Override
    public boolean buttonUp(Controller controller, int buttonCode) {
        return false;
    }

    @Override
    public boolean axisMoved(Controller controller, int axisCode, float value) {
        if (axisCode == 2) {
            mTheta = value * MAX_THETA;
        } else if (axisCode == 4) {
            mThrust = (1 - value) * MAX_THRUST;
        }

        return true;
    }

    @Override
    public boolean povMoved(Controller controller, int povCode, PovDirection value) {
        return false;
    }

    @Override
    public boolean xSliderMoved(Controller controller, int sliderCode, boolean value) {
        return false;
    }

    @Override
    public boolean ySliderMoved(Controller controller, int sliderCode, boolean value) {
        return false;
    }

    @Override
    public boolean accelerometerMoved(Controller controller, int accelerometerCode, Vector3 value) {
        return false;
    }

    @Override
    public boolean keyDown(int keycode) {
        return false;
    }

    @Override
    public boolean keyUp(int keycode) {
        return false;
    }

    @Override
    public boolean keyTyped(char character) {
        return false;
    }

    @Override
    public boolean touchDown(int screenX, int screenY, int pointer, int button) {
        return false;
    }

    @Override
    public boolean touchUp(int screenX, int screenY, int pointer, int button) {
        return false;
    }

    @Override
    public boolean touchDragged(int screenX, int screenY, int pointer) {
        return false;
    }

    @Override
    public boolean mouseMoved(int screenX, int screenY) {
        return false;
    }

    @Override
    public boolean scrolled(int amount) {
        return false;
    }

}
