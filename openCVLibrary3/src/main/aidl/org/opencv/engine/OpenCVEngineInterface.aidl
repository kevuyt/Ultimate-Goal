package org.opencv.engine;

/**
* Class provides a Java interface for org.firstinspires.ftc.teamcode.OpenCV Engine Service. It's synchronous with native OpenCVEngine class.
*/
interface OpenCVEngineInterface
{
    /**
    * @return Returns service version.
    */
    int getEngineVersion();

    /**
    * Finds an installed org.firstinspires.ftc.teamcode.OpenCV library.
    * @param org.firstinspires.ftc.teamcode.OpenCV version.
    * @return Returns path to org.firstinspires.ftc.teamcode.OpenCV native libs or an empty string if org.firstinspires.ftc.teamcode.OpenCV can not be found.
    */
    String getLibPathByVersion(String version);

    /**
    * Tries to install defined version of org.firstinspires.ftc.teamcode.OpenCV from Google Play Market.
    * @param org.firstinspires.ftc.teamcode.OpenCV version.
    * @return Returns true if installation was successful or org.firstinspires.ftc.teamcode.OpenCV package has been already installed.
    */
    boolean installVersion(String version);

    /**
    * Returns list of libraries in loading order, separated by semicolon.
    * @param org.firstinspires.ftc.teamcode.OpenCV version.
    * @return Returns names of org.firstinspires.ftc.teamcode.OpenCV libraries, separated by semicolon.
    */
    String getLibraryList(String version);
}
