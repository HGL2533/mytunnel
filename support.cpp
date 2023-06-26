#include "support.h"

void makeTunnel3DModel(Eigen::MatrixXd &verts, Eigen::MatrixXi &faces, double h, const QVector<QPoint> &points, int segNum)
{
    verts.resize(0, 3);
    faces.resize(0, 3);
    verts.conservativeResize(verts.rows() + 1, 3);
    verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(0, 0, -50);
    for(int i = 0; i < segNum; i++)
    {
        verts.conservativeResize(verts.rows() + 1, 3);
        verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(points[i].x(), points[i].y(), -50);
    }
    for(int i = 0; i < segNum; i++)
    {
        verts.conservativeResize(verts.rows() + 1, 3);
        verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(points[i].x(), points[i].y(), h);
    }
    verts.conservativeResize(verts.rows() + 1, 3);
    verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(0, 0, h);
    int ptNumBottom = (verts.rows() - 2) / 2;
    //生成底面三角形
    for(int i = 1; i < ptNumBottom; i++)
    {
        int nxt = (i + 1);
        faces.conservativeResize(faces.rows() + 1, 3);
        faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(0, nxt, i);
    }
    //生成顶面的三角形for(int i = 1; i <= ptNumBottom; i++)
    for(int i = 1; i < ptNumBottom; i++)
    {
        int nxt = (i + 1);
        faces.conservativeResize(faces.rows() + 1, 3);
        faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(i + ptNumBottom, nxt + ptNumBottom, verts.rows() - 1);
    }
    for(int i = 1; i <= ptNumBottom; i++)
    {
        int nxt = (i + 1);
        if (nxt > ptNumBottom)
            nxt = 1;
        faces.conservativeResize(faces.rows() + 1, 3);
        faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(i, nxt, i + ptNumBottom);
    }
    for(int i = 1; i <= ptNumBottom; i++)
    {
        int nxt = i + 1;
        if(nxt > ptNumBottom)
            nxt = 1;
        faces.conservativeResize(faces.rows() + 1, 3);
        faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(i + ptNumBottom, nxt, nxt + ptNumBottom);
    }
    //faces.array() += 1; // obj格式原因将所有元素加1
}

void makeBaseCuboidModel(Eigen::MatrixXd& baseVerts, Eigen::MatrixXi& baseFaces, const double len, const double h, const double d, const double miny)
{
    double height = 4 * h;
    double width = 7 * d;
    double length = len;
    baseVerts.resize(8, 3);
    baseFaces.resize(12, 3);
    
    baseVerts.row(0) = Eigen::RowVector3d(-width / 2, miny, 0);
    baseVerts.row(1) = Eigen::RowVector3d(width / 2, miny, 0);
    baseVerts.row(2) = Eigen::RowVector3d(width / 2, miny + height, 0);
    baseVerts.row(3) = Eigen::RowVector3d(-width / 2, miny + height, 0);
    baseVerts.row(4) = Eigen::RowVector3d(-width / 2, miny, length);
    baseVerts.row(5) = Eigen::RowVector3d(width / 2, miny, length);
    baseVerts.row(6) = Eigen::RowVector3d(width / 2, miny + height, length);
    baseVerts.row(7) = Eigen::RowVector3d(-width / 2, miny + height, length);

    baseFaces <<
        1, 0, 3,
        1, 3, 2,
        4, 0, 1,
        4, 1, 5,
        2, 3, 7,
        2, 7, 6,
        0, 4, 3,
        4, 7, 3,
        4, 6, 7,
        5, 6, 4,
        5, 1, 6,
        1, 2, 6;
    //baseFaces.array() += 1;
}

void output(const Eigen::MatrixXd &verts, const Eigen::MatrixXi &faces, const std::string &filename)
{
    std::ofstream ofile(filename + ".off");
    ofile << "OFF" << "\n";
    ofile << verts.rows() <<
        " " << faces.rows() << " " << 0 << "\n";
    for (int i = 0; i < verts.rows(); ++i)
        ofile << verts(i, 0) << " " << verts(i, 1) << " "
        << verts(i, 2) << "\n";
    for (int i = 0; i < faces.rows(); ++i)
        ofile << "3 " << faces(i, 0)
        << " " << faces(i, 1)
        << " " << faces(i, 2) << "\n";
}

void ConvertMatrix2SurfaceMesh(const  Eigen::MatrixXd& verts, const  Eigen::MatrixXi& faces,Mesh& sm)
{
    typedef CGAL::Point_3<K> Point_3;
    sm.clear();
    for (int i = 0; i < verts.rows(); ++i)
    {
        Point_3 p(verts(i, 0), verts(i, 1), verts(i, 2));
        sm.add_vertex(p);
    }
    for (int i = 0; i < faces.rows(); ++i)
    {
        sm.add_face(CGAL::SM_Vertex_index(faces(i, 0)),
            CGAL::SM_Vertex_index(faces(i, 1)),
            CGAL::SM_Vertex_index(faces(i, 2)));
    }
}