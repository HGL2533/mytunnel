#include "support.h"


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

void makeTunnel3DModel1(Eigen::MatrixXd& verts, Eigen::MatrixXi& faces, double h, const QVector<QPointF>& qpoints)
{
    const int qpointsSize = qpoints.size();
    verts.resize(0, 3);
    faces.resize(0, 3);
    Polygon_2 polygon1;
    for (int i = 0; i < qpointsSize; ++i)
        polygon1.push_back(Point(qpoints[i].x(), qpoints[i].y()));
    polygon1.push_back(Point(0., 0.));

    CDT cdt1;
    cdt1.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(), true);
    CGAL::refine_Delaunay_mesh_2(cdt1, Criteria());
    
    //底面或顶面上有vertsNum个点
    int vertsNum = 0;
    std::map<Vertex_handle, int> mapOfHandleToIndex;
    for (auto v = cdt1.finite_vertices_begin(); v != cdt1.finite_vertices_end(); ++v)
    {
        verts.conservativeResize(verts.rows() + 1, 3);
        verts(vertsNum, 0) = v->point().x();  // 存储 x 坐标
        verts(vertsNum, 1) = v->point().y();  // 存储 y 坐标
        verts(vertsNum, 2) = 0.0;             // 存储 z 坐标
        mapOfHandleToIndex[v] = vertsNum;     //实现Vertex_handle 到 index的映射
        ++vertsNum;
    }

    int facesNum = 0;
    for (Face_handle f : cdt1.finite_face_handles())
    {
        //关键点 is_in_domain ，看文档时没看到！！！！！！  确保在constraint里生成三角形
        if (f->is_in_domain())
        {
            facesNum++;
            faces.conservativeResize(faces.rows() + 1, 3);
            faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(mapOfHandleToIndex[f->vertex(1)], mapOfHandleToIndex[f->vertex(0)], mapOfHandleToIndex[f->vertex(2)]);
        }
    }

    //面延申出到另一个底面
    for (int i = 0; i < vertsNum; ++i)
    {
        verts.conservativeResize(verts.rows() + 1, 3);
        verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(verts(i, 0), verts(i, 1), h);
    }
    for (int i = 0; i < facesNum; ++i)
    {
        faces.conservativeResize(faces.rows() + 1, 3);
        faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(faces(i, 1) + vertsNum, faces(i, 0) + vertsNum, faces(i, 2) + vertsNum);
    }
    
    const int midSegNum = 96;
    //找到底面轮廓新生成的顶点
    std::map<double, int> mapOfBottomVertXAndId;
    for (int i = 0; i < vertsNum; ++i)
    {
        if ((verts(i, 1) == 0) && (verts(i, 2) == 0))
            mapOfBottomVertXAndId[verts(i, 0)] = i;
    }

    // 轮廓上的顶点数量, -2 是因为轮廓第一个和最后一个顶点
    const int newBoundaryVertsNum = mapOfBottomVertXAndId.size() + qpointsSize - 2;

    /* 生成在隧道中部的若干段闭合轮廓，使侧面三角形变短 */
    for (int i = 1; i < midSegNum; ++i)
    {
        /* 此处进行的处理是采用map中的所有顶点而丢弃轮廓中的首尾顶点，好处是轮廓直线上的所有顶点的索引为连续的*/
        for (int j = 1; j < qpointsSize - 1; ++j)
        {
            verts.conservativeResize(verts.rows() + 1, 3);
            verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(verts(j, 0), verts(j, 1), i * h / midSegNum);
        }
        for (auto it = mapOfBottomVertXAndId.begin(); it != mapOfBottomVertXAndId.end(); ++it)
        {
                verts.conservativeResize(verts.rows() + 1, 3);
                verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(verts(it->second, 0), verts(it->second, 1), i * h / midSegNum);
        }
        std::cout << "156 verts rows: " << verts.rows() << "\n";

    }

    for (int i = 0; i < midSegNum; ++i)
    {
        if (i == 0)
        {
            std::vector<int> v1, v2;
            for (int i = 1; i < qpointsSize - 1; ++i)
            {
                v1.emplace_back(i);
            }
            for (auto m : mapOfBottomVertXAndId)
            {
                v1.emplace_back(m.second);
            }
            
            for (int i = 0; i < newBoundaryVertsNum; ++i)
            {
                v2.emplace_back(i + 2 * vertsNum);
            }

            for (int j = 0; j < newBoundaryVertsNum; ++j)
            {
                int nxt = j + 1;
                if (nxt == newBoundaryVertsNum)
                    nxt = 0;
                faces.conservativeResize(faces.rows() + 2, 3);
                faces.row(faces.rows() - 2).array() = Eigen::RowVector3i(v1[j], v1[nxt], v2[j]);
                faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(v2[nxt], v2[j], v1[nxt]);
            }
        }
        if (i == midSegNum - 1)
        {
            std::vector<int> v1, v2;
            for (int i = 1; i < qpointsSize - 1; ++i)
            {
                v1.emplace_back(i + vertsNum);
            }
            for (auto m : mapOfBottomVertXAndId)
            {
                v1.emplace_back(m.second + vertsNum);
            }
            for (auto v : v1)
                std::cout << "v " << v << "\n";
            for (int i = 0; i < newBoundaryVertsNum; ++i)
                v2.emplace_back(i + 2 * vertsNum + (midSegNum - 2) * newBoundaryVertsNum);

            for (int j = 0; j < newBoundaryVertsNum; ++j)
            {
                int nxt = j + 1;
                if (nxt == newBoundaryVertsNum)
                    nxt = 0;
                faces.conservativeResize(faces.rows() + 2, 3);
                faces.row(faces.rows() - 2).array() = Eigen::RowVector3i(v1[nxt], v1[j], v2[j]);
                faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(v2[j], v2[nxt], v1[nxt]);
            }
        }
        if(i != 0 && i != midSegNum - 1)
        {
            std::vector<int> v1, v2;
            for (int j = 0; j < newBoundaryVertsNum; ++j)
            {
                v1.emplace_back(j + 2 * vertsNum + (i - 1) * newBoundaryVertsNum);
                v2.emplace_back(j + 2 * vertsNum + i * newBoundaryVertsNum);
            }
            for (int j = 0; j < newBoundaryVertsNum; ++j)
            {
                int nxt = j + 1;
                if (nxt == newBoundaryVertsNum)
                    nxt = 0;
                faces.conservativeResize(faces.rows() + 2, 3);
                faces.row(faces.rows() - 2).array() = Eigen::RowVector3i(v1[j], v1[nxt], v2[j]);
                faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(v2[nxt], v2[j], v1[nxt]);
            }
        }
    }

}

void makeTunnel3DModel2(Eigen::MatrixXd& verts, Eigen::MatrixXi& faces, double h, const QVector<QPointF>& qpoints)
{
    const int qpointsSize = qpoints.size();
	verts.resize(0, 3);
	faces.resize(0, 3);
	Polygon_2 polygon1;
	for (int i = 0; i < qpoints.size(); ++i)
		polygon1.push_back(Point(qpoints[i].x(), qpoints[i].y()));
	CDT cdt2;
	cdt2.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(), true);
    CGAL::refine_Delaunay_mesh_2(cdt2, Criteria());

    //vertsNum记录剖分后一个面上的顶点数
	int vertsNum = 0;
	std::map<Vertex_handle, int> mapOfHandleToIndex;
	for (auto v = cdt2.finite_vertices_begin(); v != cdt2.finite_vertices_end(); ++v)
	{
		verts.conservativeResize(verts.rows() + 1, 3);
		verts(vertsNum, 0) = v->point().x();  // 存储 x 坐标
		verts(vertsNum, 1) = v->point().y();  // 存储 y 坐标
		verts(vertsNum, 2) = 0.0;             // 存储 z 坐标
		mapOfHandleToIndex[v] = vertsNum;     //实现Vertex_handle 到 index的映射
		++vertsNum;
	}

	int facesNum = 0;
	for (Face_handle f : cdt2.finite_face_handles())
	{
		facesNum++;
		faces.conservativeResize(faces.rows() + 1, 3);
		faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(mapOfHandleToIndex[f->vertex(1)], mapOfHandleToIndex[f->vertex(0)], mapOfHandleToIndex[f->vertex(2)]);
	}

    //面延申出到另一个底面
    for (int i = 0; i < vertsNum; ++i)
    {
        verts.conservativeResize(verts.rows() + 1, 3);
        verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(verts(i, 0), verts(i, 1), h);
    }
    for (int i = 0; i < facesNum; ++i)
    {
        faces.conservativeResize(faces.rows() + 1, 3);
        faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(faces(i, 1) + vertsNum, faces(i, 0) + vertsNum, faces(i, 2) + vertsNum);
    }

    /*找到直线段上新加入的顶点*/
    std::map<double, int> mapOfBottomVertXAndId;
    std::map<double, int> mapOfLeftVertYAndId;
    std::map<double, int> mapOfRightVertYAndId;
    for (int i = 0; i < vertsNum; ++i)
    {
        if ((verts(i, 1) == 0) && (verts(i, 2) == 0))
            mapOfBottomVertXAndId[verts(i, 0)] = i;
        if ((i > 99) && (verts(i, 0) <= qpoints[100].x()))
            mapOfRightVertYAndId[verts(i, 1)] = i;
        if (verts(i, 0) >= qpoints[0].x())
            mapOfLeftVertYAndId[verts(i, 1)] = i;
    }
    

    //设置分段数
    const int midSegNum = 96;
    const int  newBoundaryVertsNum = qpointsSize + mapOfBottomVertXAndId.size() + mapOfLeftVertYAndId.size() + mapOfRightVertYAndId.size() - 6;

	/*生成在隧道中部的若干段闭合轮廓，使侧面三角形变短;  记录轮廓上的顶点id */
    std::vector<int> vId;
	for (int i = 1; i < midSegNum; ++i)
	{
		/* 此处进行的处理是采用map中的所有顶点而丢弃轮廓中的首尾顶点，好处是轮廓直线上的所有顶点的索引为连续的*/
        for (auto &ml : mapOfLeftVertYAndId)
        {
            if(i == 1)
                vId.emplace_back(ml.second);
            verts.conservativeResize(verts.rows() + 1, 3);
            verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(verts(ml.second, 0), verts(ml.second, 1), i * h / midSegNum);
        }

        for (int j = 2; j < qpointsSize - 2; ++j)
        {
            if(i == 1)
                vId.emplace_back(j);
            verts.conservativeResize(verts.rows() + 1, 3);
            verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(verts(j, 0), verts(j, 1), i * h / midSegNum);
        }

        auto mr = mapOfRightVertYAndId.end();
        mr--;
        for (; mr != mapOfRightVertYAndId.begin(); mr--)
        {
            if(i == 1)
                vId.emplace_back(mr->second);
            verts.conservativeResize(verts.rows() + 1, 3);
            verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(verts(mr->second, 0), verts(mr->second, 1), i * h / midSegNum);
        }

        auto mbStop = mapOfBottomVertXAndId.end();
        mbStop--;
        for (auto mb = mapOfBottomVertXAndId.begin(); mb != mbStop; mb++)
        {
            if(i == 1)
                vId.emplace_back(mb->second);
            verts.conservativeResize(verts.rows() + 1, 3);
            verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(verts(mb->second, 0), verts(mb->second, 1), i * h / midSegNum);
        }
	}

    /* 连接侧面*/
    for (int j = 0; j < midSegNum; ++j)
    {
        if (j == 0)
        {
            std::vector<int> v1;
            for (int i = 0; i < newBoundaryVertsNum; ++i)
                v1.emplace_back(i + 2 * vertsNum);
            for (int i = 0; i < newBoundaryVertsNum; ++i)
            {
                int nxt = i + 1;
                if (nxt == newBoundaryVertsNum)
                    nxt = 0;
                faces.conservativeResize(faces.rows() + 2, 3);
                faces.row(faces.rows() - 2).array() = Eigen::RowVector3i(vId[nxt], v1[i], vId[i]);
                faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(v1[nxt], v1[i], vId[nxt]);
            }
        }
        if (j == midSegNum - 1)
        {
            std::vector<int> v1;
            for (int i = 0; i < newBoundaryVertsNum; ++i)
                v1.emplace_back(i + 2 * vertsNum + (midSegNum - 2) * newBoundaryVertsNum);
            for (int i = 0; i < newBoundaryVertsNum; ++i)
            {
                int nxt = i + 1;
                if (nxt == newBoundaryVertsNum)
                    nxt = 0;
                faces.conservativeResize(faces.rows() + 2, 3);
                faces.row(faces.rows() - 2).array() = Eigen::RowVector3i(v1[i], vId[nxt] + vertsNum, vId[i] + vertsNum);
                faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(v1[i], v1[nxt], vId[nxt] + vertsNum);
            }
        }
        if (j != 0 && j != midSegNum - 1)
        {
            std::vector<int> v1, v2;
            for (int i = 0; i < newBoundaryVertsNum; ++i)
            {
                v1.emplace_back(i + 2 * vertsNum + (j - 1) * newBoundaryVertsNum);
                v2.emplace_back(i + 2 * vertsNum + j * newBoundaryVertsNum);
            }
            for (int i = 0; i < newBoundaryVertsNum; ++i)
            {
                int nxt = i + 1;
                if (nxt == newBoundaryVertsNum)
                    nxt = 0;
                faces.conservativeResize(faces.rows() + 2, 3);
                faces.row(faces.rows() - 2).array() = Eigen::RowVector3i(v1[i], v1[nxt], v2[i]);
                faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(v2[nxt], v2[i], v1[nxt]);
            }
        }
    }


}

void makeTunnel3DModel3(Eigen::MatrixXd& verts, Eigen::MatrixXi& faces, double h, const QVector<QPointF>& qpoints)
{
    const int qpointsSize = qpoints.size();
    verts.resize(0, 3);
    faces.resize(0, 3);
    Polygon_2 polygon3;
    for (int i = 0; i < qpoints.size(); ++i)
        polygon3.push_back(Point(qpoints[i].x(), qpoints[i].y()));

    CDT cdt3;
    cdt3.insert_constraint(polygon3.vertices_begin(), polygon3.vertices_end(), true);
    
    //CGAL::make_conforming_Delaunay_2(cdt3);
    CGAL::refine_Delaunay_mesh_2(cdt3, Criteria());
    

    int vertsNum = 0;
    std::map<Vertex_handle, int> mapOfHandleToIndex;
    for (auto v = cdt3.finite_vertices_begin(); v != cdt3.finite_vertices_end(); ++v)
    {
        verts.conservativeResize(verts.rows() + 1, 3);
        verts(vertsNum, 0) = v->point().x();  // 存储 x 坐标
        verts(vertsNum, 1) = v->point().y();  // 存储 y 坐标
        verts(vertsNum, 2) = 0.0;             // 存储 z 坐标
        mapOfHandleToIndex[v] = vertsNum;     //实现Vertex_handle 到 index的映射
        ++vertsNum;
    }


    int facesNum = 0;
    for (Face_handle f : cdt3.finite_face_handles())
    {
        if (f->is_in_domain())
        {
            facesNum++;
            faces.conservativeResize(faces.rows() + 1, 3);
            faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(mapOfHandleToIndex[f->vertex(1)], mapOfHandleToIndex[f->vertex(0)], mapOfHandleToIndex[f->vertex(2)]);
        }

    }

    //判断新添加的点是否在i----nxt 线段上，若在则判断与i的距离，然后决定他们的相对位置
    std::vector<int> newVertsInBoundary;
    for (int i = 0; i < qpoints.size(); ++i)
    {
        newVertsInBoundary.emplace_back(i);
        int nxt = i + 1;
        if (nxt == qpoints.size())
            nxt = 0;
        
        std::vector<int> temp;
        std::cout << " qpointssize: " << qpoints.size() << " bertnum: " << vertsNum << "\n";
        for (int j = qpoints.size(); j < vertsNum; ++j)
        {
            if (pointInLineSegment(i, nxt, j, verts))
            {
                temp.emplace_back(j);
            }
        }
        std::map<double, int> mapOfDistanceAndMidPointId;
        for (int a = 0; a < temp.size(); ++a)
        {
            mapOfDistanceAndMidPointId[distanceOfTwoPoints(i, temp[a], verts)] = temp[a];
        }
        for (auto mit = mapOfDistanceAndMidPointId.begin(); mit != mapOfDistanceAndMidPointId.end(); mit++)
        {
            newVertsInBoundary.emplace_back(mit->second);
        }
    }

    //面延申出到另一个底面
    for (int i = 0; i < vertsNum; ++i)
    {
        verts.conservativeResize(verts.rows() + 1, 3);
        verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(verts(i, 0), verts(i, 1), h);
    }
    for (int i = 0; i < facesNum; ++i)
    {
        faces.conservativeResize(faces.rows() + 1, 3);
        faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(faces(i, 1) + vertsNum, faces(i, 0) + vertsNum, faces(i, 2) + vertsNum);
    }
    
    const int midSegNum = 48;
    const int newBoundaryVertsNum = newVertsInBoundary.size();

    std::cout << "vertsNum: " << vertsNum << "\n";
    std::cout << "newBoundaryVertsNum: " << newBoundaryVertsNum << "\n";
    
    for (int i = 1; i < midSegNum; ++i)
    {
        for (int j = 0; j < newBoundaryVertsNum; ++j)
        {
            verts.conservativeResize(verts.rows() + 1, 3);
            verts.row(verts.rows() - 1).array() = Eigen::RowVector3d(verts(newVertsInBoundary[j], 0), verts(newVertsInBoundary[j], 1), i * h / midSegNum);
        }
    }
    /* 连接侧面*/
    for (int j = 0; j < midSegNum; ++j)
    {
        if (j == 0)
        {
            std::vector<int> v1;
            for (int i = 0; i < newBoundaryVertsNum; ++i)
                v1.emplace_back(i + 2 * vertsNum);
            for (int i = 0; i < newBoundaryVertsNum; ++i)
            {
                int nxt = i + 1;
                if (nxt == newBoundaryVertsNum)
                    nxt = 0;
                faces.conservativeResize(faces.rows() + 2, 3);
                faces.row(faces.rows() - 2).array() = Eigen::RowVector3i(newVertsInBoundary[i], newVertsInBoundary[nxt], v1[i]);
                faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(newVertsInBoundary[nxt], v1[nxt], v1[i]);
            }
        }
        if (j == midSegNum - 1)
        {
            std::vector<int> v1;
            for (int i = 0; i < newBoundaryVertsNum; ++i)
                v1.emplace_back(i + 2 * vertsNum + (midSegNum - 2) * newBoundaryVertsNum);
            for (int i = 0; i < newBoundaryVertsNum; ++i)
            {
                int nxt = i + 1;
                if (nxt == newBoundaryVertsNum)
                    nxt = 0;
                faces.conservativeResize(faces.rows() + 2, 3);
                faces.row(faces.rows() - 2).array() = Eigen::RowVector3i(newVertsInBoundary[i] + vertsNum, v1[i], newVertsInBoundary[nxt] + vertsNum);
                faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(newVertsInBoundary[nxt] + vertsNum, v1[i], v1[nxt]);
            }
        }
        if (j != 0 && j != midSegNum - 1)
        {
            std::vector<int> v1, v2;
            for (int i = 0; i < newBoundaryVertsNum; ++i)
            {
                v1.emplace_back(i + 2 * vertsNum + (j - 1) * newBoundaryVertsNum);
                v2.emplace_back(i + 2 * vertsNum + j * newBoundaryVertsNum);
            }
            for (int i = 0; i < newBoundaryVertsNum; ++i)
            {
                int nxt = i + 1;
                if (nxt == newBoundaryVertsNum)
                    nxt = 0;
                faces.conservativeResize(faces.rows() + 2, 3);
                faces.row(faces.rows() - 2).array() = Eigen::RowVector3i(v1[i], v1[nxt], v2[i]);
                faces.row(faces.rows() - 1).array() = Eigen::RowVector3i(v2[nxt], v2[i], v1[nxt]);
            }
        }
    }
}

double distanceOfTwoPoints(int id1, int id2, Eigen::MatrixXd& verts)
{
    double dis;
    double x = verts(id1, 0) - verts(id2, 0);
    double y = verts(id1, 1) - verts(id2, 1);
    dis = x * x + y * y;
    return dis;
}

bool pointInLineSegment(int id1, int id2, int id3, Eigen::MatrixXd& verts)
{
    double dx1 = verts(id2, 0) - verts(id1, 0);
    double dy1 = verts(id2, 1) - verts(id1, 1);
    double dx2 = verts(id3, 0) - verts(id1, 0);
    double dy2 = verts(id3, 1) - verts(id1, 1);
    double crossProduct = dx1 * dy2 - dx2 * dy1;
    if (crossProduct == 0)
        return true;
    else
        return false;
}