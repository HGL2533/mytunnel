#include "widget.h"

#include <QApplication>


float windowwidth = 1200;
float windowheight = 800;
int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    Widget w;

    w.resize(windowwidth,windowheight);
    w.show();

    return a.exec();
}
