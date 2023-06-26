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

//����segNumΪ�����Ķ��������Է�������Ϳ�������������ģ�����������  ���1��2 �����3����
void makeTunnel3DModel(Eigen::MatrixXd &verts, Eigen::MatrixXi &faces, double h, const QVector<QPoint> &points, int segNum);
//len�ǻ��峤����ĳ��ȣ�����ĳ��ȣ� hΪ�����ֱ����߶ȣ� dΪ����ں����ֱ���� minyΪ�������͵��y����
void makeBaseCuboidModel(Eigen::MatrixXd& baseVerts, Eigen::MatrixXi& baseFaces, const double len, const double h, const double d, const double miny);
void output(const Eigen::MatrixXd &verts, const Eigen::MatrixXi &faces, const std::string &filename);


void ConvertMatrix2SurfaceMesh(const  Eigen::MatrixXd& verts, const  Eigen::MatrixXi& faces,
    Mesh& sm);