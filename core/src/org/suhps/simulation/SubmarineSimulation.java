package org.suhps.simulation;

import com.badlogic.gdx.*;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.glutils.*;
import com.badlogic.gdx.math.*;
import com.badlogic.gdx.physics.box2d.*;

public class SubmarineSimulation extends ApplicationAdapter implements InputProcessor {

    private static final String TAG = "SUHPS";

    private static final float THETA = 10f;  // degrees

    private static final float SUB_WIDTH = 2.2f;
    private static final float SUB_HEIGHT = 0.6f;
    private static final float SUB_MASS = 140f;
    private static final float SUB_CROSS_SECTIONAL_AREA = MathUtils.PI * 0.3f * 0.3f;
    private static final float SUB_DENSITY = 1000f;
    private static final float SUB_DRAG_COEFFICIENT = 0.04f;
    private static final float SUB_LIFT_COEFFICIENT_SLOPE = 2 * MathUtils.PI;
    private static final float SUB_SPINNING_DRAG_COEFFICIENT = 0f;
    private static final float SUB_INITIAL_SPEED = 7f;

    private World mWorld;
    private Body mSubmarine;

    private OrthographicCamera mCamera;
    private ShapeRenderer mShapeRenderer;
    private Box2DDebugRenderer mRenderer;

    private boolean mPaused = true;
    private int mFrameNumber = 0;

    @Override
    public void create() {
        Box2D.init();

        //Gdx.app.setLogLevel(Application.LOG_DEBUG);
        Gdx.app.setLogLevel(Application.LOG_INFO);

        mCamera = new OrthographicCamera(Gdx.graphics.getWidth(),
                                         Gdx.graphics.getHeight());
        mCamera.zoom = 0.01f;

        mWorld = new World(new Vector2(0, 0), false);

        mRenderer = new Box2DDebugRenderer();
        mShapeRenderer = new ShapeRenderer();

        mSubmarine = createSubmarine();
        mSubmarine.setLinearVelocity(SUB_INITIAL_SPEED, 0f);

        Gdx.input.setInputProcessor(this);
    }

    private Body createSubmarine() {
        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyDef.BodyType.DynamicBody;
        bodyDef.position.set(0, 0);

        Body body = mWorld.createBody(bodyDef);

        PolygonShape shape = new PolygonShape();
        shape.setAsBox(SUB_WIDTH / 2f, SUB_HEIGHT / 2f);

        float area = SUB_WIDTH * SUB_HEIGHT;
        float density = SUB_MASS / area;

        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = shape;
        fixtureDef.density = density;
        fixtureDef.friction = 0f;
        fixtureDef.restitution = 0f;

        Fixture fixture = body.createFixture(fixtureDef);

        shape.dispose();

        return body;
    }

    private void drawForce(Vector2 position, Vector2 value) {
        Vector2 start = position;
        Vector2 end = Vector2.X.set(value).scl(0.01f).add(position);
        mShapeRenderer.x(position, 0.1f);
        mShapeRenderer.line(position, end);
    }

    private void applyThrust() {
        Vector2 thrust = new Vector2(300f, 0);
        thrust.rotate(THETA);
        thrust.rotateRad(mSubmarine.getAngle());

        Vector2 position = mSubmarine.getWorldPoint(new Vector2(-SUB_WIDTH / 2f, 0f));

        Gdx.app.debug(TAG, "Thrust = " + thrust);

        if (!mPaused) {
            mSubmarine.applyForce(thrust, position, true);
        }

        mShapeRenderer.setColor(1f, 0f, 0f, 1f);
        drawForce(position, thrust);
    }

    private void applyDrag() {
        Vector2 velocity = mSubmarine.getLinearVelocity();

        float v2 = velocity.len() * velocity.len();
        float value = 0.5f * SUB_DENSITY * SUB_CROSS_SECTIONAL_AREA * SUB_DRAG_COEFFICIENT * v2;
        Vector2 drag = velocity.cpy().nor().scl(-value);

        Gdx.app.debug(TAG, "Velocity = " + velocity + ", Drag = " + drag);

        if (!mPaused) {
            mSubmarine.applyForceToCenter(drag, true);
        }

        Vector2 position = mSubmarine.getWorldCenter().cpy();

        mShapeRenderer.setColor(0f, 1f, 0f, 1f);
        drawForce(position, drag);
    }

    private void applyLift() {
        Vector2 velocity = mSubmarine.getLinearVelocity();
        float angle = mSubmarine.getAngle();

        float alpha = angle - velocity.angleRad();

        if (Math.abs(alpha) < MathUtils.degreesToRadians * 15) {
            float liftCoefficient = alpha * SUB_LIFT_COEFFICIENT_SLOPE;

            float v2 = velocity.len() * velocity.len();
            float value = 0.5f * SUB_DENSITY * SUB_CROSS_SECTIONAL_AREA * liftCoefficient * v2;

            Vector2 lift = velocity.cpy().nor().rotate90(1).scl(value);

            Gdx.app.debug(TAG, "Velocity = " + velocity + ", Lift = " + lift);

            Vector2 position = mSubmarine.getWorldPoint(new Vector2(SUB_WIDTH / 4f, 0f));

            if (!mPaused) {
                mSubmarine.applyForce(lift, position, true);
            }

            mShapeRenderer.setColor(0f, 0f, 1f, 1f);
            drawForce(position, lift);
        }
    }

    private void applySpinningDrag() {
        float angularVelocity = mSubmarine.getAngularVelocity();

        float v2 = angularVelocity * angularVelocity;
        float value = (0.5f * SUB_DENSITY * SUB_CROSS_SECTIONAL_AREA * SUB_SPINNING_DRAG_COEFFICIENT * v2) / SUB_WIDTH;

        float drag = -value;

        if (!mPaused) {
            mSubmarine.applyTorque(drag, true);
        }

        Gdx.app.debug(TAG, "Angular Velocity = " + angularVelocity + ", Spinning Drag = " + drag);
    }

    @Override
    public void render() {
        Gdx.gl.glClearColor(1, 1, 0.95f, 1);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        Vector2 position = mSubmarine.getWorldCenter();
        mCamera.update();

        mShapeRenderer.setProjectionMatrix(mCamera.combined);

        mShapeRenderer.begin(ShapeRenderer.ShapeType.Line);

        if (!mPaused) {
            mWorld.step(1 / 60f, 6, 2);
        }

        mRenderer.render(mWorld, mCamera.combined);

        applyThrust();
        applyDrag();
        applyLift();
        applySpinningDrag();

        mShapeRenderer.end();

        if (!mPaused) {
            System.out.println(mFrameNumber + "," + position.x + "," + position.y + "," + mSubmarine.getAngle() * MathUtils.radiansToDegrees);

            mFrameNumber += 1;
        }

        try {
            Thread.sleep(100);
        } catch (Exception e) {

        }
    }

    @Override
    public boolean keyDown(int keycode) {
        return false;
    }

    @Override
    public boolean keyUp(int keycode) {
        if (keycode == Input.Keys.P) {
            mPaused = !mPaused;
            return true;
        } else {
            return false;
        }
    }

    @Override
    public boolean keyTyped(char character) {
        return false;
    }

    @Override
    public boolean touchDown(int x, int y, int pointer, int button) {
        return false;
    }

    @Override
    public boolean touchUp(int x, int y, int pointer, int button) {
        return false;
    }

    @Override
    public boolean touchDragged(int x, int y, int pointer) {
        return false;
    }

    @Override
    public boolean mouseMoved(int x, int y) {
        return false;
    }

    @Override
    public boolean scrolled(int amount) {
        return false;
    }

}
