#pragma once

#include <QWidget>

void calculateIntersectionPoints(const QPointF &p1, const QPointF &p2, const float &r1, const float &r2, QPointF &upIntersection);
//calculateIntersectionPoints 重载
void calculateIntersectionPoints(const QPointF &p1, const QPointF &p2, const float &r1, const float &r2, QPointF &downIntersection, QPointF &upIntersection);



class TunnelTemplate1{
public:
    TunnelTemplate1()
        :_x1(0.0f), _y1(28.0f), _r1(308.0f),
         _x2(80.0f), _y2(64.0f), _r2(240.0f)
         //_x3(40), _y3(20), _r3(160)
    {}

    void generate_point(QVector<QPointF> &);
    void getCenter(QPoint &Center1, QPoint &Center2, QPoint &Center3);

    void updateTunnel1(const float &deltaX, const float &deltaY, const float &deltaR1, const float &deltaR2); //整体位置的修改的和半径的修改
    void updateTunnel1Part(const float &deltaX, const float &deltaY); //修改左、右圆的位置

    float get_x1() {return _x1;}
    float get_x2() {return _x2;}
    float get_y1() {return _y1;}
    float get_y2() {return _y2;}
    float get_r1() {return _r1;}
    float get_r2() {return _r2;}
private:
    //O1为中间的圆心，O2为左边,O3与O2对称
    float _x1, _y1, _r1;
    float _x2, _y2, _r2;

};


class TunnelTemplate2{
public:
    TunnelTemplate2()
        :_x(0.0f), _y(0.0f), _r(200.0f), _h(200.0f)
    {}

    void generate_point(QVector<QPointF>&);
    void getCenter(QPoint& Center) {Center = QPoint(_x, _y);}

    //传入改变量
    void updateTunnel2(const float &x, const float &y, const float &r, const float &h) {_x += x, _y += y, _r += r, _h += h;}
    //获取private的一系列接口
    float get_x() {return _x;}
    float get_y() {return _y;}
    float get_r() {return _r;}
    float get_h() {return _h;}
    /*//修改private的一系列接口
    void set_x(float &newx) {_x = newx;}
    void set_y(float &newy) {_y = newy;}
    void set_r(float &newr) {_r = newr;}
    void set_h(float &newh) {_h = newh;}
    */
private:
    float _x, _y, _r, _h;
};

class TunnelTemplate3{
public:
    TunnelTemplate3()
        :_x1(0.0f), _y1(0.0f), _r1(260.0f),
         _x3(80.0f), _y3(-20.0f), _r3(190.0f),
         _x4(0.0f), _y4(190.0f), _r4(410.0f)
    {}
    void generate_points(QVector<QPointF>& points);
    void getCenter(QVector<QPoint>& Centers);

    //整体位置和某个半径的修改
    void updateTunnel3(const float &deltaX, const float &deltaY, const float &deltaR1, const float &deltaR3, const float &deltaR4);
    //重载用来修改O4的半径或者位置,注意位置的x坐标不能随意修改，与O1一致
    void updateTunnel3(const float &deltaY, const float &deltaR4);
    //重载用来修改O2 O3的半径或者位置
    void updateTunnel3(const float &deltaX, const float &deltaY, const float &deltaR);

    float get_x1() {return _x1;}
    float get_y1() {return _y1;}
    float get_r1() {return _r1;}
    float get_r3() {return _r3;}
    float get_r4() {return _r4;}

private:
    float _x1, _y1, _r1; //O1和O4的x坐标相同
    //O2和O3关于O1的x所在直线对称
    float _x3, _y3, _r3;
    float _x4, _y4, _r4;

};
