#pragma once

#include <iostream>
#include <iterator>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "widget.h"
#include <fstream>
#include <sstream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_conformer_2.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel   K;
typedef CGAL::Surface_mesh<K::Point_3>                        Mesh;

typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CDT::Vertex_handle Vertex_handle;
typedef CDT::Point Point;
typedef CGAL::Polygon_2<K>                                        Polygon_2;
typedef CDT::Face_handle                                          Face_handle;

namespace PMP = CGAL::Polygon_mesh_processing;





//len是基体长方体的长度，延申的长度； h为隧道竖直方向高度， d为隧道在横向的直径， miny为隧道的最低点的y坐标
void makeBaseCuboidModel(Eigen::MatrixXd& baseVerts, Eigen::MatrixXi& baseFaces, const double len, const double h, const double d, const double miny);
void output(const Eigen::MatrixXd &verts, const Eigen::MatrixXi &faces, const std::string &filename);


void ConvertMatrix2SurfaceMesh(const  Eigen::MatrixXd& verts, const  Eigen::MatrixXi& faces, Mesh& sm);



void makeTunnel3DModel1(Eigen::MatrixXd& verts, Eigen::MatrixXi& faces, double h, const QVector<QPointF>& qpoints);
void makeTunnel3DModel2(Eigen::MatrixXd& verts, Eigen::MatrixXi& faces, double h, const QVector<QPointF>& qpoints);
void makeTunnel3DModel3(Eigen::MatrixXd& verts, Eigen::MatrixXi& faces, double h, const QVector<QPointF>& qpoints);

double distanceOfTwoPoints(int id1, int id2, Eigen::MatrixXd& verts);
bool pointInLineSegment(int id1, int id2, int id3, Eigen::MatrixXd& verts);