#include "tunnel.h"
#include <iostream>
const int numSegments = 100;  //定义圆弧的分段数

void TunnelTemplate1::getCenter(QPoint &Center1, QPoint &Center2, QPoint &Center3)
{
    Center1 = QPoint(_x1, _y1);
    Center3 = QPoint(_x2, _y2);
    Center2 = QPoint(2*_x1 - _x2, _y2);
}

void TunnelTemplate1::generate_point(QVector<QPoint> & points)
{
    QPointF intersectionPointUp;
    QPoint temp;
    QVector<QPoint> tempPoints;
    calculateIntersectionPoints(QPointF(_x1, _y1), QPointF(_x2, _y2), _r1, _r2, intersectionPointUp);
    if(points.size())
        points.clear();
    const float angle2start = -asinf((_y2 - _y1)/_r2);  //y = _y1与 2圆圆心到隧道起点的夹角
    const float angle2end = acosf((intersectionPointUp.x() - _x2)/_r2);  //第一段终点
    const float angle2Step = (angle2end - angle2start) / numSegments;
    for(int i = 0; i < numSegments; i++)
    {
        float angle = angle2start + i * angle2Step;  // angle的起点是angle2start + i * angle2Step;
        float x = _x2 + _r2 * cos(angle);
        float y = _y2 + _r2 * sin(angle);
        points.push_back(QPoint(x,y));
        tempPoints.push_back(QPoint(2*_x1 - x,y));
    }

    const float angle1start = acosf((intersectionPointUp.x() - _x1)/_r1);
    const float angle1end = M_PI / 2;
    const float angle1Step = (angle1end - angle1start) / numSegments;
    for(int i = 0; i < numSegments; i++)
    {
        float angle = angle1start + i * angle1Step;
        float x = _x1 + _r1 * cos(angle);
        float y = _y1 + _r1 * sin(angle);
        points.push_back(QPoint(x,y));
        tempPoints.push_back(QPoint(2*_x1 - x,y));
    }
    while(tempPoints.size())
    {
        temp = tempPoints.back();
        points.push_back(temp);
        tempPoints.pop_back();
    }
}

void TunnelTemplate1::updateTunnel1(const float &deltaX, const float &deltaY, const float &deltaR1, const float &deltaR2)
{
    _x1 += deltaX;
    _y1 += deltaY;
    _x2 += deltaX;
    _y2 += deltaY;
    _r1 += deltaR1;
    _r2 += deltaR2;

}
void TunnelTemplate1::updateTunnel1Part(const float &deltaX, const float &deltaY)
{
    _x2 += deltaX;
    _y2 += deltaY;
}


void TunnelTemplate2::generate_point(QVector<QPoint>& points)
{
    //注意，每次调用generate_point需要把points已有数据清空
    if(points.size())
        points.clear();
    const float angleStep = M_PI / numSegments;
    points.push_back(QPoint(_x + _r, _y - _h));
    for (int i = 0; i <= numSegments; ++i)
    {
        float angle = angleStep * i;
        float x = _x + _r * cos(angle);
        float y = _y + _r * sin(angle);
        points.push_back(QPoint(x, y));
    }
        points.push_back(QPoint(_x - _r, _y - _h));
}

//传入两个圆，求得y大一些的那个交点
void calculateIntersectionPoints(const QPointF &p1, const QPointF &p2, const float &r1, const float &r2, QPointF &upIntersection)
{
    float a1, b1, R1, a2, b2, R2;
    QPointF q1, q2;
    a1 = p1.x();
    b1 = p1.y();
    R1 = r1;

    a2 = p2.x();
    b2 = p2.y();
    R2 = r2;

    float R1R1 = R1*R1;
    float a1a1 = a1*a1;
    float b1b1 = b1*b1;

    float a2a2 = a2*a2;
    float b2b2 = b2*b2;
    float R2R2 = R2*R2;

    float subs1 = a1a1 - 2 * a1*a2 + a2a2 + b1b1 - 2 * b1*b2 + b2b2;
    float subs2 = -R1R1 * a1 + R1R1 * a2 + R2R2 * a1 - R2R2 * a2 + a1a1*a1 - a1a1 * a2 - a1*a2a2 + a1*b1b1 - 2 * a1*b1*b2 + a1*b2b2 + a2a2*a2 + a2*b1b1 - 2 * a2*b1*b2 + a2*b2b2;
    float subs3 = -R1R1 * b1 + R1R1 * b2 + R2R2 * b1 - R2R2 * b2 + a1a1*b1 + a1a1 * b2 - 2 * a1*a2*b1 - 2 * a1*a2*b2 + a2a2 * b1 + a2a2 * b2 + b1b1*b1 - b1b1 * b2 - b1*b2b2 + b2b2*b2;
    float sigma = sqrt((R1R1 + 2 * R1*R2 + R2R2 - a1a1 + 2 * a1*a2 - a2a2 - b1b1 + 2 * b1*b2 - b2b2)*(-R1R1 + 2 * R1*R2 - R2R2 + subs1));
    if(std::abs(subs1)>0.0000001)//分母不为0
    {
        q1.setX((subs2 - sigma*b1 + sigma*b2) / (2 * subs1));
        q2.setX((subs2 + sigma*b1 - sigma*b2) / (2 * subs1));

        q1.setY((subs3 + sigma*a1 - sigma*a2) / (2 * subs1));
        q2.setY((subs3 - sigma*a1 + sigma*a2) / (2 * subs1));
    }
    if(q1.y() > q2.y()) {upIntersection.setX(q1.x()), upIntersection.setY(q1.y());}
    else {upIntersection.setX(q2.x()), upIntersection.setY(q2.y());}
}
void calculateIntersectionPoints(const QPointF &p1, const QPointF &p2, const float &r1, const float &r2, QPointF &downIntersection, QPointF&upIntersection)
{
    upIntersection = QPointF(0,0); //没有用的一句
    float a1, b1, R1, a2, b2, R2;
    QPointF q1, q2;
    a1 = p1.x();
    b1 = p1.y();
    R1 = r1;

    a2 = p2.x();
    b2 = p2.y();
    R2 = r2;

    float R1R1 = R1*R1;
    float a1a1 = a1*a1;
    float b1b1 = b1*b1;

    float a2a2 = a2*a2;
    float b2b2 = b2*b2;
    float R2R2 = R2*R2;

    float subs1 = a1a1 - 2 * a1*a2 + a2a2 + b1b1 - 2 * b1*b2 + b2b2;
    float subs2 = -R1R1 * a1 + R1R1 * a2 + R2R2 * a1 - R2R2 * a2 + a1a1*a1 - a1a1 * a2 - a1*a2a2 + a1*b1b1 - 2 * a1*b1*b2 + a1*b2b2 + a2a2*a2 + a2*b1b1 - 2 * a2*b1*b2 + a2*b2b2;
    float subs3 = -R1R1 * b1 + R1R1 * b2 + R2R2 * b1 - R2R2 * b2 + a1a1*b1 + a1a1 * b2 - 2 * a1*a2*b1 - 2 * a1*a2*b2 + a2a2 * b1 + a2a2 * b2 + b1b1*b1 - b1b1 * b2 - b1*b2b2 + b2b2*b2;
    float sigma = sqrt((R1R1 + 2 * R1*R2 + R2R2 - a1a1 + 2 * a1*a2 - a2a2 - b1b1 + 2 * b1*b2 - b2b2)*(-R1R1 + 2 * R1*R2 - R2R2 + subs1));
    if(std::abs(subs1)>0.0000001)//分母不为0
    {
        q1.setX((subs2 - sigma*b1 + sigma*b2) / (2 * subs1));
        q2.setX((subs2 + sigma*b1 - sigma*b2) / (2 * subs1));

        q1.setY((subs3 + sigma*a1 - sigma*a2) / (2 * subs1));
        q2.setY((subs3 - sigma*a1 + sigma*a2) / (2 * subs1));
    }
    if(q1.y() > q2.y()) {downIntersection.setX(q2.x()), downIntersection.setY(q2.y());}
    else {downIntersection.setX(q1.x()), downIntersection.setY(q1.y());}
}


void TunnelTemplate3::generate_points(QVector<QPoint>& points)
{
    if(points.size())
        points.clear();
    QPointF intersectionPointUp34, intersectionPointUp13; //34,13分别代表O3、O4交点，O1、O3交点
    QPoint temp;
    QVector<QPoint> tempPoints;
    calculateIntersectionPoints(QPointF(_x3, _y3), QPointF(_x4, _y4), _r3, _r4, intersectionPointUp34, intersectionPointUp34);
    calculateIntersectionPoints(QPointF(_x1, _y1), QPointF(_x3, _y3), _r1, _r3, intersectionPointUp13, intersectionPointUp13);

    std::cout << intersectionPointUp13.x() <<"  " << intersectionPointUp13.y() << "\n";
    std::cout << intersectionPointUp34.x() <<"  " << intersectionPointUp34.y() << "\n";

    const float angle34start = -M_PI/2;
    const float angle34end = -asinf((_y4 - intersectionPointUp34.y()) / _r4);
    const float angle34Step = (angle34end - angle34start) / numSegments;
    for(int i = 0; i < numSegments; i++)
    {
        float angle = angle34start + angle34Step * i;
        float x = _x4 + _r4 * cos(angle);
        float y = _y4 + _r4 * sin(angle);
        points.push_back(QPoint(x, y));
        tempPoints.push_back(QPoint(2 * _x1 - x, y));
    }
    const float angle13start = -asinf((_y3 - intersectionPointUp34.y()) / _r3);
    const float angle13end = -asinf((_y3 - intersectionPointUp13.y()) / _r3);
    const float angle13Step = (angle13end - angle13start) / numSegments;
    for(int i = 0; i < numSegments; i++)
    {
        float angle = angle13start + angle13Step * i;
        float x = _x3 + _r3 * cos(angle);
        float y = _y3 + _r3 * sin(angle);
        points.push_back(QPoint(x, y));
        tempPoints.push_back(QPoint(2 * _x1 - x, y)); //关于O1 O4圆心所在直线对称
    }
    const float angle11start = -asinf((_y1 - intersectionPointUp13.y()) / _r1);
    const float angle11end = M_PI / 2;
    const float angle11Step = (angle11end - angle11start) / numSegments;
    for(int i = 0; i < numSegments; i++)
    {
        float angle = angle11start + angle11Step * i;
        float x = _x1 + _r1 * cos(angle);
        float y = _y1 + _r1 * sin(angle);
        points.push_back(QPoint(x, y));
        tempPoints.push_back(QPoint(2 * _x1 - x, y));
    }
    while(tempPoints.size())
    {
        temp = tempPoints.back();
        points.push_back(temp);
        tempPoints.pop_back();
    }
}

void TunnelTemplate3::getCenter(QVector<QPoint> &Centers)
{
    if(Centers.size())
        Centers.clear();
    Centers.push_back(QPoint(_x1, _y1)); //O1对应center0
    Centers.push_back(QPoint(2 * _x4 - _x3, _y3));
    Centers.push_back(QPoint(_x3, _y3));
    Centers.push_back(QPoint(_x4, _y4));
}

void TunnelTemplate3::updateTunnel3(const float &deltaX, const float &deltaY, const float &deltaR1, const float &deltaR3, const float &deltaR4)
{
    _x1 += deltaX;
    _x3 += deltaX;
    _x4 += deltaX;
    _y1 += deltaY;
    _y3 += deltaY;
    _y4 += deltaY;
    _r1 += deltaR1;
    _r3 += deltaR3;
    _r4 += deltaR4;
}

void TunnelTemplate3::updateTunnel3(const float &deltaY, const float &deltaR4)
{
    _y4 += deltaY;
    _r4 += deltaR4;
}

void TunnelTemplate3::updateTunnel3(const float &deltaX, const float &deltaY, const float &deltaR)
{
    _x3 += deltaX;
    _y3 += deltaY;
    _r3 += deltaR;
}
