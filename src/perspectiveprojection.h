#ifndef PERSPECTIVEPROJECTION_H
#define PERSPECTIVEPROJECTION_H

///Rzutowanie perspektywiczne na plaszczyzne ekranu
///Tu jest to opisane i sa wykorzystywane rownania https://en.wikipedia.org/wiki/3D_projection

class PerspectiveProjection
{
	private:
        double alfa, beta, gamma;//katy kamery (alfa - os x, beta - os y, gamma - os z)
        double f; //ogniskowa
        cv::Point3f T; //T - pozycja

    public:
        View(double f1, cv::Point3f angles, cv::Point3f position);

        void setF(double f1) {f = f1;};
        void setAngles(cv::Point3f angles);
        void setPosition(cv::Point3f pos) {T = pos;};

        void rotate(cv::Point3f angles);
        void translate(cv::Point3f pos);

        cv::Point2f projectPoint(cv::Point3f point) const; // rzutuje punkt na daną płaszczyznę
};

#endif