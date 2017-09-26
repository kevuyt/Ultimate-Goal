/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package Library4997.MasqExternal;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.vuforia.Matrix34F;
import com.vuforia.TrackableResult;
import com.vuforia.VuMarkTarget;
import com.vuforia.VuMarkTargetResult;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaPoseMatrix;

import java.util.HashMap;
import java.util.Map;

/**
 * {@link MasqVuforiaListener} is the default listener used for {@link VuforiaTrackable}
 * implementations. This listener facilitates polling for results of the tracking. (Advanced:) Alternate
 * listeners could make use of event-driven results by taking actions in the {@link VuforiaTrackable.Listener}
 * methods.
 *
 * @see VuforiaTrackable
 * @see VuforiaTrackable#getListener()
 */
@SuppressWarnings("WeakerAccess")
public class MasqVuforiaListener implements VuforiaTrackable.Listener {
    public final static String TAG = "Vuforia";

    protected VuforiaTrackable trackable;
    protected boolean newPoseAvailable;
    protected boolean newLocationAvailable;
    protected Matrix34F currentPose;
    protected Matrix34F lastTrackedPose;
    protected MasqVuMark vuMarkInstanceId = null;
    protected OpenGLMatrix phoneLocationOnRobotInverted;
    protected VuforiaLocalizer.CameraDirection cameraDirection;
    protected final Map<VuforiaLocalizer.CameraDirection, OpenGLMatrix> poseCorrectionMatrices;

    public MasqVuforiaListener(VuforiaTrackable trackable) {
        this.trackable = trackable;
        this.newPoseAvailable = false;
        this.newLocationAvailable = false;
        this.currentPose = this.lastTrackedPose = null;
        this.vuMarkInstanceId = null;
        this.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.poseCorrectionMatrices = new HashMap<VuforiaLocalizer.CameraDirection, OpenGLMatrix>();
        this.poseCorrectionMatrices.put(VuforiaLocalizer.CameraDirection.BACK, new OpenGLMatrix(new float[]{
                        0, -1,  0,  0,
                        -1,  0,  0,  0,
                        0,  0, -1,  0,
                        0,  0,  0,  1
                }));
        this.poseCorrectionMatrices.put(VuforiaLocalizer.CameraDirection.FRONT, new OpenGLMatrix(new float[]{
                        0,  1,  0,  0,
                        -1,  0,  0,  0,
                        0,  0,  1,  0,
                        0,  0,  0,  1
                }));
    }
    public synchronized void setPhoneInformation(@NonNull OpenGLMatrix phoneLocationOnRobot, @NonNull VuforiaLocalizer.CameraDirection cameraDirection) {
        this.phoneLocationOnRobotInverted = phoneLocationOnRobot.inverted();
        this.cameraDirection = cameraDirection;
    }

    public synchronized OpenGLMatrix getPhoneLocationOnRobot() {
        return this.phoneLocationOnRobotInverted == null ? null : this.phoneLocationOnRobotInverted.inverted();
    }

    public synchronized VuforiaLocalizer.CameraDirection getCameraDirection() {
        return this.cameraDirection;
    }

    public synchronized @Nullable OpenGLMatrix getRobotLocation() {
        // Capture the location in order to avoid races with concurrent updates
        OpenGLMatrix trackableLocationOnField = trackable.getLocation();
        OpenGLMatrix pose = this.getPose();
        if (pose != null && trackableLocationOnField != null && this.phoneLocationOnRobotInverted != null) {
            OpenGLMatrix result =
                    trackableLocationOnField
                            .multiplied(pose.inverted())
                            .multiplied(phoneLocationOnRobotInverted);
            return result;
        }
        else
            return null;
    }

    public @NonNull OpenGLMatrix getPoseCorrectionMatrix(VuforiaLocalizer.CameraDirection direction)
    {
        synchronized (this.poseCorrectionMatrices) {
            return this.poseCorrectionMatrices.get(direction);
        }
    }

    /** @see #getPoseCorrectionMatrix(VuforiaLocalizer.CameraDirection) */
    public void setPoseCorrectionMatrix(VuforiaLocalizer.CameraDirection direction, @NonNull OpenGLMatrix matrix) {
        synchronized (this.poseCorrectionMatrices) {
            this.poseCorrectionMatrices.put(direction, matrix);
        }
    }

    public synchronized OpenGLMatrix getUpdatedRobotLocation() {
        if (this.newLocationAvailable) {
            this.newLocationAvailable = false;
            return getRobotLocation();
        }
        else
            return null;
    }

    public synchronized @Nullable OpenGLMatrix getPose() {
        OpenGLMatrix pose = getRawPose();
        return pose==null ? null : this.getPoseCorrectionMatrix(this.cameraDirection).multiplied(pose);
    }

    public synchronized @Nullable OpenGLMatrix getRawPose()
    {
        if (this.currentPose != null) {
            OpenGLMatrix result = new VuforiaPoseMatrix(this.currentPose).toOpenGL();
            // RobotLog.vv(TAG, "rawPose -> %s", result.toString());
            return result;
        }
        else
            return null;
    }

    public synchronized @Nullable OpenGLMatrix getRawUpdatedPose() {
        if (this.newPoseAvailable) {
            this.newPoseAvailable = false;
            return getRawPose();
        }
        else
            return null;
    }

    public boolean isVisible()
    {
        return getPose() != null;
    }

    public synchronized OpenGLMatrix getLastTrackedRawPose() {
        return this.lastTrackedPose == null ? null : new VuforiaPoseMatrix(this.lastTrackedPose).toOpenGL();
    }

    public synchronized @Nullable
    MasqVuMark getVuMarkInstanceId() {
        return vuMarkInstanceId;
    }

    public synchronized void onTracked(TrackableResult trackableResult, VuforiaTrackable child) {
        this.currentPose = trackableResult.getPose();
        this.newPoseAvailable = true;
        this.newLocationAvailable = true;
        this.lastTrackedPose = this.currentPose;

        if (trackableResult.isOfType(VuMarkTargetResult.getClassType()))
        {
            VuMarkTargetResult vuMarkTargetResult = (VuMarkTargetResult)trackableResult;
            VuMarkTarget vuMarkTarget = (VuMarkTarget) vuMarkTargetResult.getTrackable();
            vuMarkInstanceId = new MasqVuMark(vuMarkTarget.getInstanceId());
        }
    }

    public void onTracked(TrackableResult trackableResult) {

    }

    @Override public synchronized void onNotTracked() {
        this.currentPose = null;
        this.newPoseAvailable = true;
        this.newLocationAvailable = true;
        this.vuMarkInstanceId = null;
    }
}
