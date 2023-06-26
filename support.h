#ifndef SUPPORT_H
#define SUPPORT_H

#endif // SUPPORT_H

#include <Eigen/Dense>
#include "widget.h"
#include <fstream>
#include <sstream>
#include <string>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <iostream>
#include <string>
typedef CGAL::Exact_predicates_inexact_constructions_kernel   K;
typedef CGAL::Surface_mesh<K::Point_3>                        Mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

//参数segNum为轮廓的段数，所以封闭轮廓和开放轮廓有区别的！！！！！！  情况1，2 和情况3传入
void makeTunnel3DModel(Eigen::MatrixXd &verts, Eigen::MatrixXi &faces, double h, const QVector<QPoint> &points, int segNum);
//len是基体长方体的长度，延申的长度； h为隧道竖直方向高度， d为隧道在横向的直径， miny为隧道的最低点的y坐标
void makeBaseCuboidModel(Eigen::MatrixXd& baseVerts, Eigen::MatrixXi& baseFaces, const double len, const double h, const double d, const double miny);
void output(const Eigen::MatrixXd &verts, const Eigen::MatrixXi &faces, const std::string &filename);


void ConvertMatrix2SurfaceMesh(const  Eigen::MatrixXd& verts, const  Eigen::MatrixXi& faces,
    Mesh& sm);