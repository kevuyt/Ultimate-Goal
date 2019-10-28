package org.opencv.android;

/**
 * Interface for callback object in case of asynchronous initialization of org.firstinspires.ftc.teamcode.OpenCV.
 */
public interface LoaderCallbackInterface
{
    /**
     * org.firstinspires.ftc.teamcode.OpenCV initialization finished successfully.
     */
    static final int SUCCESS = 0;
    /**
     * Google Play Market cannot be invoked.
     */
    static final int MARKET_ERROR = 2;
    /**
     * org.firstinspires.ftc.teamcode.OpenCV library installation has been canceled by the user.
     */
    static final int INSTALL_CANCELED = 3;
    /**
     * This version of org.firstinspires.ftc.teamcode.OpenCV Manager Service is incompatible with the app. Possibly, a service update is required.
     */
    static final int INCOMPATIBLE_MANAGER_VERSION = 4;
    /**
     * org.firstinspires.ftc.teamcode.OpenCV library initialization has failed.
     */
    static final int INIT_FAILED = 0xff;

    /**
     * Callback method, called after org.firstinspires.ftc.teamcode.OpenCV library initialization.
     * @param status status of initialization (see initialization status constants).
     */
    public void onManagerConnected(int status);

    /**
     * Callback method, called in case the package installation is needed.
     * @param callback answer object with approve and cancel methods and the package description.
     */
    public void onPackageInstall(final int operation, InstallCallbackInterface callback);
};
