#include "widget.h"
#include "ui_widget.h"

extern float windowwidth;
extern float windowheight;


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
}

Widget::~Widget()
{
    delete ui;
}


void Widget::on_pushButton1_clicked()
{
    drawFlag2 = false;
    drawFlag3 = false;
    drawFlag1 = true;
    update();
}

void Widget::on_pushButton2_clicked()
{
    drawFlag1 = false;
    drawFlag3 = false;
    drawFlag2 = true;
    update();
}

void Widget::on_pushButton3_clicked()
{
    drawFlag1 = false;
    drawFlag2 = false;
    drawFlag3 = true;
    update();
}

void Widget::on_pushButton4_clicked()
{
    if(drawFlag1)
    {
        for(int i = 0; i < widgetPoints1.size(); i++)
        {
            //将隧道的中心平移到 0 0 点处
            widgetPoints1[i].setX(widgetPoints1[i].x() - _tunnel1.get_x1());
            widgetPoints1[i].setY(widgetPoints1[i].y() - _tunnel1.get_y1());
        }
        double tunnelHeight = widgetPoints1[200].y();
        double tunnelWidth = 2 * widgetPoints1[0].x();
        double miny = widgetPoints1[0].y();
        makeBaseCuboidModel(baseVerts, baseFaces, 10000, tunnelHeight, tunnelWidth, miny);

        QVector<QPoint> widgetLessPoints1;
        for (int i = 0; i < 40; i++)
        {
            widgetLessPoints1.push_back(widgetPoints1[i * 10]);
        }
        widgetLessPoints1.push_back(widgetPoints1[widgetPoints1.size() - 1]);
        makeTunnel3DModel(verts, faces, 10500, widgetLessPoints1, 41);
        //makeTunnel3DModel(verts, faces, 1050, widgetPoints1, 400);


        Mesh mesh1, mesh2;
        ConvertMatrix2SurfaceMesh(verts, faces, mesh2);
        ConvertMatrix2SurfaceMesh(baseVerts, baseFaces, mesh1);

        /*std::string filename1 = "out111";
        std::string filename2 = "out222";
        output(baseVerts, baseFaces, filename1);
        output(verts, faces, filename2);
        PMP::IO::read_polygon_mesh("out111.off", mesh1);
        PMP::IO::read_polygon_mesh("out222.off", mesh2);*/

        Mesh result1;
        PMP::corefine_and_compute_difference(mesh1, mesh2, result1);
        CGAL::IO::write_polygon_mesh("result1.off", result1, CGAL::parameters::stream_precision(17));
   
    }
    if(drawFlag2)
    {
        
        float offsetX = (widgetPoints2[0].x() + widgetPoints2[widgetPoints2.size() - 1].x()) / 2.0;
        float offsetY = (widgetPoints2[0].y() + widgetPoints2[widgetPoints2.size() - 1].y()) / 2.0;
        for(int i = 0; i < widgetPoints2.size(); i++)
        {
            //0 0 点为隧道底边中点
            widgetPoints2[i].setX(widgetPoints2[i].x() - offsetX);
            widgetPoints2[i].setY(widgetPoints2[i].y() - offsetY);
        }
        double tunnelHeight = widgetPoints2[widgetPoints2.size() / 2].y() - widgetPoints2[0].y();
        double tunnelWidth = widgetPoints2[0].x() - widgetPoints2[widgetPoints2.size() - 1].x();
        double miny = widgetPoints2[0].y();
        makeBaseCuboidModel(baseVerts, baseFaces, 6000, tunnelHeight, tunnelWidth, miny);
        makeTunnel3DModel(verts, faces, 6050, widgetPoints2, widgetPoints2.size());
        output(verts, faces, "2outtunnel");
        output(baseVerts, baseFaces, "2outbase");

        Mesh mesh1, mesh2;
        ConvertMatrix2SurfaceMesh(baseVerts, baseFaces, mesh1);
        ConvertMatrix2SurfaceMesh(verts, faces, mesh2);
        Mesh result2;
        PMP::corefine_and_compute_difference(mesh1, mesh2, result2);
        CGAL::IO::write_polygon_mesh("result2.off", result2, CGAL::parameters::stream_precision(17));
    }
    if(drawFlag3)
    {
        float offsetX = widgetPoints3[0].x();
        float offsetY = widgetPoints3[0].y();
        for(int i = 0; i < widgetPoints3.size(); i++)
        {
            // 0 0 为隧道y值最小的点
            widgetPoints3[i].setX(widgetPoints3[i].x() - offsetX);
            widgetPoints3[i].setY(widgetPoints3[i].y() - offsetY);
        }
        double tunnelHeight = widgetPoints3[widgetPoints3.size() / 2].y() - widgetPoints3[0].y();
        double tunnelWidth = widgetPoints3[200].x() - widgetPoints3[400].x();
        double miny = widgetPoints3[0].y();
        makeBaseCuboidModel(baseVerts, baseFaces, 6000, tunnelHeight, tunnelWidth, miny);

        QVector<QPoint> widgetLessPoints3;
        for (int i = 0; i < widgetPoints3.size()/10; i++)
        {
            widgetLessPoints3.push_back(widgetPoints3[i * 10]);
        }
        makeTunnel3DModel(verts, faces, 6050, widgetLessPoints3, widgetLessPoints3.size()+1);

        //makeTunnel3DModel(verts, faces, 6050, widgetPoints3, widgetPoints3.size());
        output(verts, faces, "3outtunnel");
        output(baseVerts, baseFaces, "3outbase");

        Mesh mesh1, mesh2;
        ConvertMatrix2SurfaceMesh(baseVerts, baseFaces, mesh1);
        ConvertMatrix2SurfaceMesh(verts, faces, mesh2);
        Mesh result3;
        PMP::corefine_and_compute_difference(mesh1, mesh2, result3);
        CGAL::IO::write_polygon_mesh("result3.off", result3, CGAL::parameters::stream_precision(17));
    }
}


void Widget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    QPen pen(Qt::SolidLine);
    painter.setPen(pen);

    int centerX = width() / 2;
    int centerY = height() / 2;
    // 设置坐标变换，将绘图的中心点移动到窗口的中心点
    painter.translate(centerX, centerY);
    painter.scale(1, -1);   //反转y轴方向，现为右x正，上y正
    std::cout << "pait!@@#@";
    if(drawFlag1)
    {
        _tunnel1.generate_point(widgetPoints1);
        _tunnel1.getCenter(tunnel1Center1, tunnel1Center2, tunnel1Center3);
        // 将顶点坐标连接起来
        painter.drawPolyline(widgetPoints1.data(), widgetPoints1.size());
        painter.setPen(Qt::DashLine);
        painter.drawLine(tunnel1Center3.x(), tunnel1Center3.y(), widgetPoints1[50].x(), widgetPoints1[50].y());
        painter.drawLine(tunnel1Center1.x(), tunnel1Center1.y(), widgetPoints1[200].x(), widgetPoints1[200].y());
        painter.drawLine(tunnel1Center2.x(), tunnel1Center2.y(), widgetPoints1[350].x(), widgetPoints1[350].y());
        QFont font;
        font.setPointSize(8);
        painter.setFont(font);
        QString labeltext1 = QString("R:%1").arg(_tunnel1.get_r1());
        QString labeltext2 = QString("R:%1").arg(_tunnel1.get_r2());
        int labelx1 = tunnel1Center1.x()+5;
        int labely1 = -(tunnel1Center1.y() + widgetPoints1[200].y())/2;
        int labelx2 = (tunnel1Center2.x() + widgetPoints1[350].x())/2;
        int labely2 = -(tunnel1Center2.y() + widgetPoints1[350].y())/2;
        int labelx3 = (tunnel1Center3.x() + widgetPoints1[50].x())/2;
        int labely3 = -(tunnel1Center3.y() + widgetPoints1[50].y())/2;
        painter.scale(1, -1);
        painter.drawText(labelx1, labely1, labeltext1);
        painter.drawText(labelx2, labely2, labeltext2);
        painter.drawText(labelx3, labely3, labeltext2);
        painter.scale(1, -1);
        // 设置圆心的颜色为红色,且设置点的大小
        pen.setWidth(5);
        pen.setColor(Qt::red);
        painter.setPen(pen);
        painter.drawPoint(tunnel1Center1);
        painter.drawPoint(tunnel1Center2);
        painter.drawPoint(tunnel1Center3);

    }
    if(drawFlag2)
    {
        _tunnel2.generate_point(widgetPoints2);
        _tunnel2.getCenter(tunnel2Center);
        painter.drawPolyline(widgetPoints2.data(), widgetPoints2.size());
        //以下为标注辅助线和label
        //半径绘制文字时，由于之前反转了y的正方向，因而要先将y反转，再drawtext，text的y坐标也应该取相反数
        painter.setPen(Qt::DashLine);
        painter.drawLine(_tunnel2.get_x(), _tunnel2.get_y(), widgetPoints2[31].x(), widgetPoints2[31].y());
        QFont font;
        font.setPointSize(8);
        painter.setFont(font);
        QString labeltext1 = QString("R:%1").arg(_tunnel2.get_r());
        QString labeltext2 = QString("H:%1").arg(_tunnel2.get_h());
        int labelx1 = (_tunnel2.get_x() + widgetPoints2[31].x())/2 + 5;
        int labely1 = -(_tunnel2.get_y() + widgetPoints2[31].y())/2;
        int labelx2 = widgetPoints2[0].x() + 5;
        int labely2 = -(widgetPoints2[0].y() + widgetPoints2[1].y())/2;
        painter.scale(1, -1);
        painter.drawText(labelx1, labely1, labeltext1);
        painter.drawText(labelx2, labely2, labeltext2);
        painter.scale(1, -1);
        // 设置圆心的颜色为红色,且设置点的大小
        pen.setWidth(5);
        pen.setColor(Qt::red);
        painter.setPen(pen);
        painter.drawPoint(tunnel2Center);
        painter.drawPoint(widgetPoints2[0]);

    }
    if(drawFlag3)
    {
        _tunnel3.generate_points(widgetPoints3);
        _tunnel3.getCenter(tunnel3Centers);
        painter.drawPolyline(widgetPoints3.data(), widgetPoints3.size());
        painter.setPen(Qt::DashLine);
        painter.drawLine(tunnel3Centers[0].x(), tunnel3Centers[0].y(), widgetPoints3[200].x(), widgetPoints3[200].y());
        painter.drawLine(tunnel3Centers[0].x(), tunnel3Centers[0].y(), widgetPoints3[400].x(), widgetPoints3[400].y());
        painter.drawLine(tunnel3Centers[1].x(), tunnel3Centers[1].y(), widgetPoints3[400].x(), widgetPoints3[400].y());
        painter.drawLine(tunnel3Centers[1].x(), tunnel3Centers[1].y(), widgetPoints3[500].x(), widgetPoints3[500].y());
        painter.drawLine(tunnel3Centers[2].x(), tunnel3Centers[1].y(), widgetPoints3[100].x(), widgetPoints3[100].y());
        painter.drawLine(tunnel3Centers[2].x(), tunnel3Centers[1].y(), widgetPoints3[200].x(), widgetPoints3[200].y());
        painter.drawLine(tunnel3Centers[3].x(), tunnel3Centers[3].y(), widgetPoints3[100].x(), widgetPoints3[100].y());
        painter.drawLine(tunnel3Centers[3].x(), tunnel3Centers[3].y(), widgetPoints3[500].x(), widgetPoints3[500].y());
        // 设置圆心的颜色为红色,且设置点的大小
        pen.setWidth(5);
        pen.setColor(Qt::red);
        painter.setPen(pen);
        painter.drawPoint(tunnel3Centers[0]);
        painter.drawPoint(tunnel3Centers[1]);
        painter.drawPoint(tunnel3Centers[2]);
        painter.drawPoint(tunnel3Centers[3]);

        QFont font;
        font.setPointSize(8);
        painter.setFont(font);
        QString labeltext1 = QString("R:%1").arg(_tunnel3.get_r1());
        QString labeltext2 = QString("R:%1").arg(_tunnel3.get_r3());
        QString labeltext3 = QString("R:%1").arg(_tunnel3.get_r4());
        int labelx1 = (tunnel3Centers[0].x() + widgetPoints3[200].x()) / 2;
        int labely1 = -(tunnel3Centers[0].y() + widgetPoints3[200].y()) / 2 + 5;
        int labelx2 = (tunnel3Centers[1].x() + widgetPoints3[400].x()) / 2 - 30;
        int labely2 = -(tunnel3Centers[1].y() + widgetPoints3[400].y()) / 2;
        int labelx3 = (tunnel3Centers[2].x() + widgetPoints3[200].x()) / 2;
        int labely3 = -(tunnel3Centers[2].y() + widgetPoints3[200].y()) / 2;
        int labelx4 = (tunnel3Centers[3].x() + widgetPoints3[100].x()) / 2;
        int labely4 = -(tunnel3Centers[3].y() + widgetPoints3[100].y()) / 2;
        painter.scale(1, -1);
        painter.drawText(labelx1, labely1, labeltext1);
        painter.drawText(labelx2, labely2, labeltext2);
        painter.drawText(labelx3, labely3, labeltext2);
        painter.drawText(labelx4, labely4, labeltext3);
        painter.scale(1, -1);
        /*painter.drawPoint(widgetPoints3[0]);
        painter.drawPoint(widgetPoints3[100]);
        painter.drawPoint(widgetPoints3[400]);
        painter.drawPoint(widgetPoints3[200]);
        painter.drawPoint(widgetPoints3[500]);*/
    }
}



void Widget::mousePressEvent(QMouseEvent *event)
{
    std::cout << "mousePressEvent" << std::endl;
    if(drawFlag1)
    {
        if(event->button() == Qt::LeftButton)
        {
            float pointsize = 5;
            int x = tunnel1Center1.x();
            int y = tunnel1Center1.y();
            int x2 = tunnel1Center2.x();
            int y2 = tunnel1Center2.y();
            int x3 = tunnel1Center3.x();
            int y3 = tunnel1Center3.y();
            if((event->pos().x() - width()/2 >= x - pointsize/2)
                    && (event->pos().x() - width()/2 <= x + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y))
            {
                //整体移动控制
                dragging = true;
                dragStartPosition = event->pos();
            }
            if((event->pos().x() - width()/2 >= x2 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x2 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y2)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y2))
            {
                //左圆移动控制
                draggingPart1 = true;
                dragStartPosition = event->pos();
            }
            if((event->pos().x() - width()/2 >= x3 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x3 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y3)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y3))
            {
                //右圆移动控制
                draggingPart2 = true;
                dragStartPosition = event->pos();
            }
        }
        if(event->button() == Qt::RightButton)
        {
            //检测是否右键选取了圆心
            float pointsize = 5;
            int x1 = tunnel1Center1.x();
            int y1 = tunnel1Center1.y();
            int x2 = tunnel1Center2.x();
            int y2 = tunnel1Center2.y();
            int x3 = tunnel1Center3.x();
            int y3 = tunnel1Center3.y();
            if((event->pos().x() - width()/2 >= x1 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x1 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y1)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y1))
            {
                changeTunnel1Radius1 = true;
            }
            if((event->pos().x() - width()/2 >= x2 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x2 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y2)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y2))
            {
                changeTunnel1Radius2 = true;
            }
            if((event->pos().x() - width()/2 >= x3 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x3 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y3)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y3))
            {
                changeTunnel1Radius2 = true;
            }
        }
    }
    if(drawFlag2)
    {
        if(event->button() == Qt::LeftButton)
        {
            float pointsize = 5;
            int x = tunnel2Center.x();
            int y = tunnel2Center.y();
            if((event->pos().x() - width()/2 >= x - pointsize/2)
                    && (event->pos().x() - width()/2 <= x + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y))
            {
                dragging = true;
                dragStartPosition = event->pos();
            }
        }
        if(event->button() == Qt::RightButton)  //右键选取，加滚轮一起对半径进行控制
        {
            //检测是否右键选取了圆心
            float pointsize = 5;
            int x = tunnel2Center.x();
            int y = tunnel2Center.y();
            if((event->pos().x() - width()/2 >= x - pointsize/2)
                    && (event->pos().x() - width()/2 <= x + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y))
            {
                changeTunnel2Radius = true;
            }
            int wallX = widgetPoints2[0].x();
            int wallY = widgetPoints2[0].y();
            if((event->pos().x() - width()/2 >= wallX - pointsize/2)
                    && (event->pos().x() - width()/2 <= wallX + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - wallY)
                    && (event->pos().y() - height()/2 <= pointsize/2 - wallY))
            {
                changeWallHeight = true;
                std::cout << "changeWallHeightchangeWallHeight\n";
            }
        }
    }
    if(drawFlag3)
    {
        if(event->button() == Qt::LeftButton)
        {
            float pointsize = 5;
            int x1 = tunnel3Centers[0].x();
            int y1 = tunnel3Centers[0].y();
            int x2 = tunnel3Centers[1].x();
            int y2 = tunnel3Centers[1].y();
            int x3 = tunnel3Centers[2].x();
            int y3 = tunnel3Centers[2].y();
            int x4 = tunnel3Centers[3].x();
            int y4 = tunnel3Centers[3].y();
            if((event->pos().x() - width()/2 >= x1 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x1 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y1)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y1))
            {
                //整体移动控制
                draggingWhole = true;
                dragStartPosition = event->pos();
            }
            if((event->pos().x() - width()/2 >= x2 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x2 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y2)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y2))
            {
                //左圆移动控制
                draggingO2 = true;
                dragStartPosition = event->pos();
            }
            if((event->pos().x() - width()/2 >= x3 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x3 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y3)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y3))
            {
                //右圆移动控制
                draggingO3 = true;
                dragStartPosition = event->pos();
            }
            if((event->pos().x() - width()/2 >= x4 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x4 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y4)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y4))
            {
                //右圆移动控制
                draggingO4 = true;
                dragStartPosition = event->pos();
            }
        }
        if(event->button() == Qt::RightButton)
        {
            float pointsize = 5;
            int x1 = tunnel3Centers[0].x();
            int y1 = tunnel3Centers[0].y();
            int x2 = tunnel3Centers[1].x();
            int y2 = tunnel3Centers[1].y();
            int x3 = tunnel3Centers[2].x();
            int y3 = tunnel3Centers[2].y();
            int x4 = tunnel3Centers[3].x();
            int y4 = tunnel3Centers[3].y();
            if((event->pos().x() - width()/2 >= x1 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x1 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y1)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y1))
            {
                changeTunnel3Radius1 = true;
            }
            if((event->pos().x() - width()/2 >= x2 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x2 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y2)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y2))
            {
                changeTunnel3Radius3 = true;
            }
            if((event->pos().x() - width()/2 >= x3 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x3 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y3)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y3))
            {
                changeTunnel3Radius3 = true;
            }
            if((event->pos().x() - width()/2 >= x4 - pointsize/2)
                    && (event->pos().x() - width()/2 <= x4 + pointsize/2)
                    && (event->pos().y() - height()/2 >= -pointsize/2 - y4)
                    && (event->pos().y() - height()/2 <= pointsize/2 - y4))
            {
                changeTunnel3Radius4 = true;
            }
        }
    }
}
void Widget::wheelEvent(QWheelEvent *event)
{
    if(drawFlag1)
    {
        if(changeTunnel1Radius1 || changeTunnel1Radius2)
        {
            QPoint numPixel = event->pixelDelta()/8;
            QPoint numDegree = event->angleDelta()/8;
            int delta = 0;
            if(!numPixel.isNull())
            {
                delta = numPixel.y();
            }
            else if(!numDegree.isNull())
            {
                QPoint numsteps = numDegree/15;
                delta = numsteps.y();
            }
            changeTunnel1Radius1 == true ? _tunnel1.updateTunnel1(0, 0, delta, 0) : _tunnel1.updateTunnel1(0, 0, 0, delta);
            update();
        }
    }
    if(drawFlag2)
    {
        //右键选中圆心或者直墙，滚轮可以调整半径或高度
        if(changeTunnel2Radius || changeWallHeight)
        {
            QPoint numPixel = event->pixelDelta()/8;  //控制滚轮的调节量
            QPoint numDegree = event->angleDelta()/8;
            int delta = 0;
            if(!numPixel.isNull())
            {
                delta = numPixel.y();
            }
            else if(!numDegree.isNull())
            {
                QPoint numsteps = numDegree/15;
                delta = numsteps.y();
            }
            changeTunnel2Radius == true?_tunnel2.updateTunnel2(0, 0, delta, 0):_tunnel2.updateTunnel2(0, 0, 0, delta);
            update();
        }
    }
    if(drawFlag3)
    {
        QPoint numPixel = event->pixelDelta()/8;
        QPoint numDegree = event->angleDelta()/8;
        int delta = 0;
        if(!numPixel.isNull())
        {
            delta = numPixel.y();
        }
        else if(!numDegree.isNull())
        {
            QPoint numsteps = numDegree/15;
            delta = numsteps.y();
        }
        if(changeTunnel3Radius1)
        {_tunnel3.updateTunnel3(0, 0, delta, 0, 0);}
        if(changeTunnel3Radius3)
        {_tunnel3.updateTunnel3(0, 0, 0, delta, 0);}
        if(changeTunnel3Radius4)
        {_tunnel3.updateTunnel3(0, 0, 0, 0, delta);}
        update();
    }
}
void Widget::mouseMoveEvent(QMouseEvent *event)
{
    if(drawFlag1)
    {
        if(dragging)
        {
            int offsetX = event->pos().x()  - dragStartPosition.x();
            int offsetY = event->pos().y()  - dragStartPosition.y();
            //根据偏移修改半圆和圆心
            _tunnel1.updateTunnel1(static_cast<float>(offsetX), static_cast<float>(-offsetY), 0, 0);
            std::cout << "+"<<tunnel1Center1.y() << std::endl;
            update();
            //更新鼠标下次起始点位置
            dragStartPosition = event->pos();
        }
        if(draggingPart1)
        {
            int offsetX = event->pos().x()  - dragStartPosition.x();
            int offsetY = event->pos().y()  - dragStartPosition.y();
            //根据偏移修改半圆和圆心
            _tunnel1.updateTunnel1Part(static_cast<float>(-offsetX), static_cast<float>(-offsetY));
            std::cout << "+"<<tunnel1Center1.y() << std::endl;
            update();
            //更新鼠标下次起始点位置
            dragStartPosition = event->pos();
        }
        if(draggingPart2)
        {
            int offsetX = event->pos().x()  - dragStartPosition.x();
            int offsetY = event->pos().y()  - dragStartPosition.y();
            //根据偏移修改半圆和圆心
            _tunnel1.updateTunnel1Part(static_cast<float>(offsetX), static_cast<float>(-offsetY));
            std::cout << "+"<<tunnel1Center1.y() << std::endl;
            update();
            //更新鼠标下次起始点位置
            dragStartPosition = event->pos();
        }
    }
    if(drawFlag2 && dragging)
    {
        int offsetX = event->pos().x()  - dragStartPosition.x();
        int offsetY = event->pos().y()  - dragStartPosition.y();
        //根据偏移修改半圆和圆心
        _tunnel2.updateTunnel2(static_cast<float>(offsetX), static_cast<float>(-offsetY), 0, 0);
        update();
        //更新鼠标下次起始点位置
        dragStartPosition = event->pos();
    }
    if(drawFlag3)
    {
        if(draggingWhole)
        {
            int offsetX = event->pos().x()  - dragStartPosition.x();
            int offsetY = event->pos().y()  - dragStartPosition.y();
            //根据偏移修改半圆和圆心
            _tunnel3.updateTunnel3(static_cast<float>(offsetX), static_cast<float>(-offsetY), 0, 0, 0);
            update();
            //更新鼠标下次起始点位置
            dragStartPosition = event->pos();
        }
        if(draggingO2)
        {
            int offsetX = event->pos().x()  - dragStartPosition.x();
            int offsetY = event->pos().y()  - dragStartPosition.y();
            //根据偏移修改O2的位置
            _tunnel3.updateTunnel3(static_cast<float>(-offsetX), static_cast<float>(-offsetY), 0);
            update();
            dragStartPosition = event->pos();
        }
        if(draggingO3)
        {
            int offsetX = event->pos().x()  - dragStartPosition.x();
            int offsetY = event->pos().y()  - dragStartPosition.y();
            //根据偏移修改O3的位置
            _tunnel3.updateTunnel3(static_cast<float>(offsetX), static_cast<float>(-offsetY), 0);
            update();
            dragStartPosition = event->pos();
        }
        if(draggingO4)
        {
            int offsetY = event->pos().y()  - dragStartPosition.y();
            _tunnel3.updateTunnel3(static_cast<float>(-offsetY), 0);
            update();
            dragStartPosition = event->pos();
        }
    }

}
void Widget::mouseReleaseEvent(QMouseEvent *event)
{
    if(drawFlag1)
    {
        std::cout << "mouseReleaseEvent" << std::endl;
        if (event->button() == Qt::LeftButton)
        {
            dragging = false;
            draggingPart1 = false;
            draggingPart2 = false;
        }
        if (event->button() == Qt::RightButton)
        {
            changeTunnel1Radius1 = false;
            changeTunnel1Radius2 = false;
        }
    }
    if(drawFlag2)
    {
        std::cout << "mouseReleaseEvent" << std::endl;
        if (event->button() == Qt::LeftButton)
            dragging = false;
        if(event->button() == Qt::RightButton)
        {
            changeTunnel2Radius = false;
            changeWallHeight = false;
        }
    }
    if(drawFlag3)
    {
        if(event->button() == Qt::LeftButton)
        {
            draggingWhole = false;
            draggingO2 = false;
            draggingO3 = false;
            draggingO4 = false;
        }
        if(event->button() == Qt::RightButton)
        {
            changeTunnel3Radius1 = false;
            changeTunnel3Radius3 = false;
            changeTunnel3Radius4 = false;
        }
    }
}


