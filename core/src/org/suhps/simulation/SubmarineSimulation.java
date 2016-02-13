package org.suhps.simulation;

import java.io.*;
import java.text.*;
import java.util.*;

import com.badlogic.gdx.*;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.glutils.*;
import com.badlogic.gdx.math.*;
import com.badlogic.gdx.physics.box2d.*;

public class SubmarineSimulation extends ApplicationAdapter implements InputProcessor {

    private static final String TAG = "SUHPS";

    // Properties of the submarine
    private static final float SUB_WIDTH = 2.2f;
    private static final float SUB_HEIGHT = 0.6f;
    private static final float SUB_MASS = 140f;
    private static final float SUB_CROSS_SECTIONAL_AREA = MathUtils.PI * 0.3f * 0.6f;
    private static final float SUB_DRAG_COEFFICIENT = 0.04f;
    private static final float SUB_LIFT_COEFFICIENT_SLOPE = MathUtils.PI / 2f;
    private static final float SUB_SPINNING_DRAG_COEFFICIENT = 2f;
    private static final float SUB_INITIAL_SPEED = 0f;
    private static final float SUB_INITIAL_ANGLE = MathUtils.PI;

    // Properties of the fins
    private static final float FINS_CROSS_SECTIONAL_AREA = 0.1f;
    private static final float FINS_LIFT_COEFFICIENT_SLOPE = MathUtils.PI;
    private static final float FINS_DRAG_COEFFICIENT = 0.03f;

    // Properties of the simulation
    private static final float SIM_MAX_THRUST = 150f;
    private static final float SIM_STEP_SIZE = 1 / 100f;

    // Properties of the course
    private static final float COURSE_WIDTH = 90f;
    private static final float COURSE_HEIGHT = 52f;
    private static final float FLUID_DENSITY = 1000f;

    private World mWorld;
    private Submarine mSubmarine;

    private OrthographicCamera mCamera;
    private ShapeRenderer mShapeRenderer;
    private Box2DDebugRenderer mRenderer;

    private Logger mLogger;

    private List<Body> mObstacles;

    private boolean mPaused = true;
    private int mFrameNumber = 0;

    private float mThrust = 0;
    private float mTheta = 0;

    @Override
    public void create() {
        Box2D.init();

        mLogger = new Logger();

        Gdx.app.setLogLevel(Application.LOG_DEBUG);

        mObstacles = new ArrayList<Body>();

        mCamera = new OrthographicCamera(Gdx.graphics.getWidth(),
                                         Gdx.graphics.getHeight());

        mWorld = new World(new Vector2(0, 0), false);

        mRenderer = new Box2DDebugRenderer();
        mShapeRenderer = new ShapeRenderer();

        mSubmarine = createSubmarine();

        createCourse();

        Gdx.input.setInputProcessor(this);
    }

    @Override
    public void dispose() {
        mLogger.dispose();
    }

    private Submarine createSubmarine() {
        return new Submarine(SUB_WIDTH, SUB_HEIGHT, SUB_MASS, SUB_CROSS_SECTIONAL_AREA,
                SUB_DRAG_COEFFICIENT, SUB_LIFT_COEFFICIENT_SLOPE, SUB_SPINNING_DRAG_COEFFICIENT,
                FINS_CROSS_SECTIONAL_AREA, FINS_LIFT_COEFFICIENT_SLOPE, FINS_DRAG_COEFFICIENT,
                COURSE_WIDTH / 2f - 15, COURSE_HEIGHT / 4f, SUB_INITIAL_SPEED, SUB_INITIAL_ANGLE,
                mWorld);
    }

    private void createObstacle(float x, float y) {
        CircleShape shape = new CircleShape();
        shape.setRadius(0.1f);

        BodyDef bodyDef = new BodyDef();
        bodyDef.position.set(new Vector2(x, y));

        Body body = mWorld.createBody(bodyDef);
        body.createFixture(shape, 0.0f);
        shape.dispose();

        mObstacles.add(body);
    }

    private void createWalls() {
        BodyDef bodyDef = new BodyDef();
        bodyDef.position.set(new Vector2(0, 0));

        Body body = mWorld.createBody(bodyDef);

        ChainShape shape = new ChainShape();
        shape.createLoop(new float[] {
            -COURSE_WIDTH / 2f, -COURSE_HEIGHT / 2f,
            +COURSE_WIDTH / 2f, -COURSE_HEIGHT / 2f,
            +COURSE_WIDTH / 2f, +COURSE_HEIGHT / 2f,
            -COURSE_WIDTH / 2f, +COURSE_HEIGHT / 2f
        });
        body.createFixture(shape, 0.0f);
        shape.dispose();
    }

    private void createBarrier() {
        CircleShape circleShape = new CircleShape();
        circleShape.setRadius(4f);

        BodyDef bodyDef = new BodyDef();
        bodyDef.position.set(new Vector2(0, 0));

        Body body = mWorld.createBody(bodyDef);
        body.createFixture(circleShape, 0.0f);
        circleShape.dispose();

        PolygonShape polygonShape = new PolygonShape();
        polygonShape.setAsBox(COURSE_WIDTH / 4f, 1.5f);

        bodyDef = new BodyDef();
        bodyDef.position.set(new Vector2(COURSE_WIDTH / 4f, 0));

        body = mWorld.createBody(bodyDef);
        body.createFixture(polygonShape, 0.0f);
        polygonShape.dispose();
    }

    private void createCourse() {
        createWalls();
        createBarrier();

        createObstacle(COURSE_WIDTH / 4f, COURSE_HEIGHT / 4f - 6.5f);
        createObstacle(COURSE_WIDTH / 4f, COURSE_HEIGHT / 4f + 6.5f);

        createObstacle(-COURSE_WIDTH / 4f, COURSE_HEIGHT / 4f - 6.5f);
        createObstacle(-COURSE_WIDTH / 4f, COURSE_HEIGHT / 4f + 6.5f);
        createObstacle(-COURSE_WIDTH / 6f, COURSE_HEIGHT / 4f - 6.5f);
        createObstacle(-COURSE_WIDTH / 6f, COURSE_HEIGHT / 4f + 6.5f);

        float gap = -COURSE_WIDTH / 8f;
        createObstacle(-gap, -COURSE_HEIGHT / 4f);
        createObstacle(0, -COURSE_HEIGHT / 4f);
        createObstacle(+gap, -COURSE_HEIGHT / 4f);
        createObstacle(+gap * 2f, -COURSE_HEIGHT / 4f);

        createObstacle(COURSE_WIDTH / 4f, -COURSE_HEIGHT / 4f - 6.5f);
        createObstacle(COURSE_WIDTH / 4f, -COURSE_HEIGHT / 4f + 6.5f);
    }

    @Override
    public void render() {
        Gdx.gl.glClearColor(0, 0, 0, 1);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        mShapeRenderer.begin(ShapeRenderer.ShapeType.Filled);
        mShapeRenderer.identity();
        mShapeRenderer.setColor(0.95f, 1f, 1f, 1);
        mShapeRenderer.rect(-COURSE_WIDTH / 2f, -COURSE_HEIGHT / 2f, COURSE_WIDTH, COURSE_HEIGHT);
        mShapeRenderer.end();

        Vector2 position = mSubmarine.getWorldCenter();
        mCamera.update();

        mShapeRenderer.setProjectionMatrix(mCamera.combined);

        if (!mPaused) {
            mWorld.step(SIM_STEP_SIZE, 6, 2);
        }

        mShapeRenderer.begin(ShapeRenderer.ShapeType.Filled);

        // sub
        mShapeRenderer.identity();
        mShapeRenderer.translate(position.x, position.y, 0);
        mShapeRenderer.rotate(0, 0, 1, mSubmarine.getAngle() * MathUtils.radiansToDegrees);
        mShapeRenderer.setColor(0, 0, 0, 1);
        mShapeRenderer.ellipse(-SUB_WIDTH, -SUB_HEIGHT, SUB_WIDTH * 2f, SUB_HEIGHT * 2f);

        // obstacles
        mShapeRenderer.setColor(0.5f, 0.5f, 0.5f, 1);

        for (Body body : mObstacles) {
            mShapeRenderer.identity();
            mShapeRenderer.translate(body.getWorldCenter().x, body.getWorldCenter().y, 0);
            mShapeRenderer.circle(0, 0, 0.4f, 8);
        }

        // barrier
        mShapeRenderer.identity();
        mShapeRenderer.circle(0, 0, 4f, 32);
        mShapeRenderer.rect(0, -1.5f, COURSE_WIDTH / 2f, 3f);

        mShapeRenderer.end();

        mShapeRenderer.begin(ShapeRenderer.ShapeType.Line);

        mShapeRenderer.identity();

        mSubmarine.update(mShapeRenderer, mThrust, mTheta, FLUID_DENSITY);

        mShapeRenderer.end();

        mRenderer.render(mWorld, mCamera.combined);

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

        mTheta = MathUtils.clamp(mTheta, -20f, +20f);
        mThrust = MathUtils.clamp(mThrust, 0, SIM_MAX_THRUST);

        if (!mPaused) {
            mLogger.log(mFrameNumber * SIM_STEP_SIZE, position.x, position.y,
                    mSubmarine.getAngle() * MathUtils.radiansToDegrees);

            mFrameNumber += 1;
        }
    }

    @Override
    public void resize(int w, int h) {
        Gdx.app.debug(TAG, "Resizing to " + w + "x" + h + ".");
        mCamera.setToOrtho(false);
        mCamera.position.set(0, 0, 0);

        float zoom1 = COURSE_WIDTH / w;
        float zoom2 = COURSE_HEIGHT / h;

        mCamera.zoom = Math.max(zoom1, zoom2);

        mCamera.update();
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
