cv::Point3f rotate3D(cv::Point3f pt, cv::Point3f angles)
{
    Mat Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
    Mat Ry = (Mat_<float>(3,3) << cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
    Mat Rz = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);

    Mat R = Rz*Ry*Rx;

    Mat P1 = (Mat_<float>(3,1) << pt.x, pt.y, pt.z);

    Mat P11 = R*P1;

    Point3f p1 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));

    return p1;
}