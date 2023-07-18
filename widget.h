#pragma once

#include <iostream>
#include <QWidget>
#include <QtMath>
#include <QPainter>
#include <QPushButton>
#include <QMouseEvent>
#include "tunnel.h"
#include <Eigen/Dense>
#include "support.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE


class Widget : public QWidget
{
    Q_OBJECT


public:
    explicit Widget(QWidget* parent = nullptr);
    ~Widget();



    //隧道截面延申的3d模型
    Eigen::MatrixXd verts;
    Eigen::MatrixXi faces;
    //长方体基体的模型
    Eigen::MatrixXd baseVerts;
    Eigen::MatrixXi baseFaces;


    /*2D截面编辑部分*/
    // tunnel1的部分
    QVector<QPointF> widgetPoints1;
    QPoint tunnel1Center1, tunnel1Center2, tunnel1Center3;


    // tunnel2的部分
    QVector<QPointF> widgetPoints2;
    QPoint tunnel2Center;

    //tunnel3的部分
    QVector<QPointF> widgetPoints3;
    QVector<QPoint> tunnel3Centers;


    //公用部分，记录拖拽操作的开始位置
    QPoint dragStartPosition;

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private slots:
    void on_pushButton1_clicked();

    void on_pushButton2_clicked();

    void on_pushButton3_clicked();

    void on_pushButton4_clicked();

private:
    Ui::Widget*ui;


    TunnelTemplate1 _tunnel1;
    TunnelTemplate2 _tunnel2;
    TunnelTemplate3 _tunnel3;

    //三个bool变量控制绘制不同模板的内容
    bool drawFlag1 = false;
    bool drawFlag2 = false;
    bool drawFlag3 = false;
    //控制整体是否可拖动
    bool dragging = false;

    //tunnel1的部分,其中保证对称，左右两圆的半径相同
    bool changeTunnel1Radius1 = false;
    bool changeTunnel1Radius2 = false;
    bool draggingPart1 = false; //右圆移动
    bool draggingPart2 = false; //部分移动控制(左圆的移动)

    //tunnel2的部分
    bool changeTunnel2Radius = false; //控制鼠标滚轮是否可以改变半径
    bool changeWallHeight = false; //控制鼠标滚轮是否可以改变墙高

    Eigen::MatrixXd tunnel2Verts, tunnel2Faces;

    //tunnel3的部分
    bool changeTunnel3Radius1 = false;
    bool changeTunnel3Radius3 = false;
    bool changeTunnel3Radius4 = false;
    bool draggingWhole = false; //O1圆心为整体移动控制点
    bool draggingO4 = false; //O4、O1无论怎么移动，横坐标应该相同
    bool draggingO3 = false; //O3 O2无论怎么移动，关于O1--O4对称
    bool draggingO2 = false;


    //generate部分
    bool generate1 = false;
    bool generate2 = false;
    bool generate3 = false;
};
