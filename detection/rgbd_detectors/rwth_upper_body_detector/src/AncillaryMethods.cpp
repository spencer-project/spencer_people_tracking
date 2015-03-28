#include "AncillaryMethods.h"

void AncillaryMethods::IntersetRect(const Vector<double>& rect1, const Vector<double>& rect2, Vector<double>& rectInter)
{
    double rect1_x = rect1(0);
    double rect1_y = rect1(1);
    double rect1_w = rect1(2);
    double rect1_h = rect1(3);

    double rect2_x = rect2(0);
    double rect2_y = rect2(1);
    double rect2_w = rect2(2);
    double rect2_h = rect2(3);

    double left = rect1_x > rect2_x ? rect1_x : rect2_x;
    double top = rect1_y > rect2_y ? rect1_y : rect2_y;
    double lhs = rect1_x + rect1_w;
    double rhs = rect2_x + rect2_w;
    double right = lhs < rhs ? lhs : rhs;
    lhs = rect1_y + rect1_h;
    rhs = rect2_y + rect2_h;
    double bottom = lhs < rhs ? lhs : rhs;

    rectInter.pushBack(right < left ? 0 : left);
    rectInter.pushBack(bottom < top ? 0 : top);
    rectInter.pushBack(right < left ? 0 : right - left);
    rectInter.pushBack(bottom < top ? 0 : bottom - top);
}

void AncillaryMethods::NonMinSuppression2d(Matrix<double>& distances, Vector<Vector<double> >& pos_max, double threshold)
{
    Vector<double> pos(2);

    int y_size = distances.y_size() - 1;
    int x_size = distances.x_size() - 1;

    for(int x = 1; x < x_size; x++)
    {
        for(int y = 1; y < y_size; y++)
        {
            if (distances(x,y) < threshold)
                if (distances(x,y)<distances(x,y-1))
                    if (distances(x,y)<distances(x-1,y-1))
                        if (distances(x,y)<distances(x-1,y))
                            if (distances(x,y)<distances(x-1,y+1))
                                if (distances(x,y)<distances(x,y+1))
                                    if (distances(x,y)<distances(x+1,y+1))
                                        if (distances(x,y)<distances(x+1,y))
                                            if (distances(x,y)<distances(x+1,y-1))
                                            {
                                                pos(0) = x; pos(1) = y;
                                                pos_max.pushBack(pos);
                                            }
        }
    }
}

void AncillaryMethods::GreedyNonMaxSuppression(Vector<Vector<double> >& all_boxes, double overlap_threshold, double filter_threshold, const Matrix<double>& temp,
                                               Vector<Vector<double> >& final_bboxes)
{
    Vector<Vector<double> > final_bboxes_unfiltered;
    Vector<Vector<double> > remained_boxes = all_boxes;

    // First find for each box the number of boxes with a IoU over the threshold
    while(remained_boxes.getSize() > 0)
    {
        int min_index;
        AncillaryMethods::MinVecVecDim(remained_boxes, 4, min_index);

        final_bboxes_unfiltered.pushBack(remained_boxes(min_index));

        Vector<Vector<double> > remained_boxes_tmp;
        for(int j = 0; j < remained_boxes.getSize(); j++)
        {
            Vector<double> rectInter;

            AncillaryMethods::IntersetRect(remained_boxes(min_index), remained_boxes(j), rectInter);

            if(AncillaryMethods::LevelOneIOU(temp, remained_boxes(min_index), remained_boxes(j), rectInter) < overlap_threshold)
            {
                remained_boxes_tmp.pushBack(remained_boxes(j));
            }
        }

        remained_boxes = remained_boxes_tmp;
    }

    // Filter bounding boxes by filter_threshold
    for(int i = 0; i < final_bboxes_unfiltered.getSize(); i++)
    {
        if(final_bboxes_unfiltered(i)(4) < filter_threshold)
        {
            final_bboxes.pushBack(final_bboxes_unfiltered(i));
        }
    }

    ROS_DEBUG("Reduced the Number of BBoxes from %i to %i", all_boxes.getSize(), final_bboxes_unfiltered.getSize());
    //        if(Globals::verbose)
    //          cout << "Reduced the Number of BBoxes from " << all_boxes.getSize() << " to " << final_bboxes_unfiltered.getSize() << endl;
}

Vector<double> AncillaryMethods::PlaneToCam(const Camera& camera)
{
    Vector<double> plane = camera.get_GP();

    Vector<double> pv(plane(0), plane(1), plane(2));

    Matrix<double> cam_rot_trans = Transpose(camera.get_R());

    pv = cam_rot_trans * pv;

    Vector<double> t = cam_rot_trans * camera.get_t();

    double d = plane(3) + pv(0)*t(0) + pv(1)*t(1) + pv(2)*t(2);

    Vector<double> gp_in_camera(4);
    gp_in_camera(0) = pv(0);
    gp_in_camera(1) = pv(1);
    gp_in_camera(2) = pv(2);
    gp_in_camera(3) = d;

    return gp_in_camera;
}

Vector<double> AncillaryMethods::PlaneToWorld(const Camera& camera, const Vector<double>& plane_in_camera)
{
    Vector<double> pv(plane_in_camera(0), plane_in_camera(1), plane_in_camera(2));
    pv = camera.get_R() * pv;

    double d = plane_in_camera(3) - DotProduct(pv, camera.get_t());

    Vector<double> plane(4);
    plane(0) = pv(0)/pv.norm();
    plane(1) = pv(1)/pv.norm();
    plane(2) = pv(2)/pv.norm();
    plane(3) = d;

    return plane;
}

Camera AncillaryMethods::GetCameraOrigin(const Camera& camWorld)
{
    return Camera(camWorld.get_K(), Eye<double>(3), Vector<double>(3, 0.0), AncillaryMethods::PlaneToCam(camWorld));
}

void AncillaryMethods::MinMaxVecVecDim(Vector<Vector<double> >& v, int second_dim, double& min, double& max)
{
    min = v(0)(second_dim);
    max = v(0)(second_dim);
    double current;
    for(int i = 1; i<v.getSize(); i++)
    {
        current = v(i)(second_dim);
        if(current < min)
            min = current;
        else if(current > max)
            max = current;
    }
}

void AncillaryMethods::MinMaxIndexVecVecDim(Vector<Vector<double> >& v, int second_dim, int& index_min, int& index_max)
{
    // start with max = first element
    double min = v(0)(second_dim);
    index_min = 0;
    // start with max = first element
    double max = v(0)(second_dim);
    index_max = 0;

    for(int i = 1; i<v.getSize(); i++)
    {
        if(v(i)(second_dim) < min)
        {
            min =v(i)(second_dim);
            index_min = i;
        }
        else if(v(i)(second_dim) > max)
        {
            max =v(i)(second_dim);
            index_max = i;
        }
    }
}

double AncillaryMethods::MinVecVecDim(Vector<Vector<double> >& v, int second_dim, int& index)
{
    double min = v(0)(second_dim);       // start with max = first element
    index = 0;

    for(int i = 1; i<v.getSize(); i++)
    {
        if(v(i)(second_dim) < min)
        {
            min =v(i)(second_dim);
            index = i;
        }
    }
    return min;
}

double AncillaryMethods::MaxVecVecDim(Vector<Vector<double> >& v, int second_dim, int& index)
{
    double max = v(0)(second_dim);       // start with max = first element
    index = 0;

    for(int i = 1; i<v.getSize(); i++)
    {
        if(v(i)(second_dim) > max)
        {
            max =v(i)(second_dim);
            index = i;
        }
    }
    return max;
}

double AncillaryMethods::MedianOfMatrixRejectZero(const Matrix<double>& mat, int start_row, int end_row, int start_col, int end_col)
{
    if(start_col >= end_col || start_row >= end_row)
        return 0;

    Vector<double> a((end_row-start_row)*(end_col-start_col));
    if(a.getSize()<2)
        return 0;

    int counter = 0;
    for(int i = start_col; i < end_col; i++)
    {
        for(int j = start_row; j < end_row; j++)
        {
            a(counter) = mat(i,j);
            counter++;
        }
    }

    a.sortV();

    int index = a.getSize()/2;
    return a[index];
}

double AncillaryMethods::LevelOneIOU(const Matrix<double>& temp, const Vector<double>& bbox1, const Vector<double>& bbox2, const Vector<double>& boxIntersect)
{
    // befotre calling the method make sure that the bboxes have certain overlap.
    Matrix<double> temp1 = temp;
    Matrix<double> temp2 = temp;

    // Bring the template to the same size as the resulting detection bonding box
    if(temp1.x_size() < bbox1(2))
    {
        temp1.UpSample(bbox1(2), bbox1(2));
    }else if(temp1.x_size() > bbox1(2))
    {
        temp1.DownSample(bbox1(2), bbox1(2));
    }

    if(temp2.x_size() < bbox2(2))
    {
        temp2.UpSample(bbox2(2), bbox2(2));
    }else if(temp2.x_size() > bbox2(2))
    {
        temp2.DownSample(bbox2(2), bbox2(2));
    }

    int intersection = 0;
    int Union = 0;

    int Union1 = 0;
    int Union2 = 0;

    // compute the surface of each template individually
    int temp1_ySize = temp1.y_size();
    for(int i = 0; i < temp1.x_size(); i++)
    {
        for(int j = 0; j < temp1_ySize; j++)
        {
            if(temp1(i,j) != 0)
                Union1 += 1;
        }
    }

    int temp2_ySize = temp2.y_size();
    for(int i = 0; i < temp2.x_size(); i++)
    {
        for(int j = 0; j < temp2_ySize; j++)
        {
            if(temp2(i,j) != 0)
                Union2 += 1;
        }
    }

    // compute the intersection between the both templates

    for(int i = boxIntersect(0); i < boxIntersect(0)+boxIntersect(2); i++)
    {
        int ix1 = i-bbox1(0), ix2 = i-bbox2(0);
        for(int j = boxIntersect(1); j < boxIntersect(1)+boxIntersect(3); j++)
        {
            if(temp1(ix1,j-bbox1(1)) > 0 && temp2(ix2, j-bbox2(1)))
            {
                intersection += 1;
            }
        }
    }

    Union = Union1 + Union2 - intersection;
    return intersection/(double) Union;

}

double AncillaryMethods::DistanceToPlane(const Vector<double>& point, const Vector<double>& gp)
{
    return gp(0)*point(0) + gp(1)*point(1) + gp(2)*point(2) + gp(3)*Globals::WORLD_SCALE;
}

//void AncillaryMethods::RenderBBox2DWithScore(const Vector<double>& bbox, QImage& image, int r, int g, int b, int lineWidth)
//{
//    QPainter painter(&image);

//    QColor qColor;
//    qColor.setRgb(r, g, b);

//    QPen pen;
//    pen.setColor(qColor);
//    pen.setWidth(lineWidth);

//    painter.setPen(pen);

//    int x =(int) bbox(0);
//    int y =(int) bbox(1);
//    int w =(int) bbox(2);
//    int h =(int) bbox(3);

//    painter.drawLine(x,y, x+w,y);
//    painter.drawLine(x,y, x,y+h);
//    painter.drawLine(x+w,y, x+w,y+h);
//    painter.drawLine(x,y+h, x+w,y+h);
//}

Vector<double> AncillaryMethods::getGaussian1D(double sigma, double precision)
{

    Vector<double> kernel;
    int size = 1 + 2*ceil(precision*sigma);
    kernel.setSize(size);
    int center = size/2.0;

    double x;
    double sum = 0.0;

    for (int i = 0; i < size; i++)
    {
        x = i-center;
        kernel(i) = exp(x*x/(-2.0*sigma*sigma))/(sigma*sqrt(2.0*M_PI));
        sum +=kernel(i);
    }

    for (int i = 0; i < size; i++)
    {
        kernel(i) /=sum;
    }

    return kernel;
}

Vector<double> AncillaryMethods::conv1D(const Vector<double> &vec, const Vector<double> &kernel)
{
    int kernelSize = kernel.getSize();
    int kCenter = floor(kernelSize/2.0);

    int nn;

    Vector<double> result(vec.getSize(), 0.0);

    double vec_first = vec(0), vec_last = vec(vec.getSize()-1);

    for(int j = 0; j < vec.getSize(); j++)
    {
        for (int n = 0; n < kernelSize; n++)
        {
            nn = kernelSize - 1 - n;
            int pos = j + (n - kCenter);
            if(pos<0)
            {
                result(j) += vec_first*kernel(nn);
            }
            else if(pos >= vec.getSize())
            {
                result(j) += vec_last*kernel(nn);
            }
            else//if(pos >= 0 && pos < vec.getSize())
            {
                result(j) += vec(pos)*kernel(nn);
            }
        }
    }

    return result;
}

void AncillaryMethods::MorphologyErode(Matrix<double> &img)
{
    //    int st[3][3] = {{0,1,0}, {1,1,1}, {0,1,0}};
    int x_size = img.x_size(), y_size = img.y_size();

    Matrix<double> img_extend(x_size+2, y_size+2);
    img_extend.insert(img, 1, 1);
    img_extend(0,0) = img(0,0);
    img_extend(x_size+1,0) = img(x_size-1,0);
    img_extend(0,y_size+1) = img(0,y_size-1);
    img_extend(x_size+1,y_size+1) = img(x_size-1,y_size-1);
    for(int x = 0; x<x_size; ++x)
    {
        img_extend(x+1,0) = img(x, 0);
        img_extend(x+1, y_size+1) = img(x, y_size-1);
    }
    for(int y = 0; y<y_size; ++y)
    {
        img_extend(0,y+1) = img(0,y);
        img_extend(x_size+1,y+1) = img(x_size-1,y);
    }

    for(int x=1; x<=x_size; ++x)
    {
        for(int y=1; y<=y_size; ++y)
        {
            if(img_extend(x,y)>0)
            {
                if(!(img_extend(x-1,y)>0 && img_extend(x,y-1)>0 && img_extend(x+1,y)>0 && img_extend(x,y+1)>0))
                    img(x-1,y-1)=0;
            }
        }
    }
}

void AncillaryMethods::MorphologyDilate(Matrix<double> &img)
{
    //    int st[3][3] = {{0,1,0}, {1,1,1}, {0,1,0}};
    int x_size = img.x_size(), y_size = img.y_size();

    Matrix<double> img_extend(x_size+2, y_size+2);
    img_extend.insert(img, 1, 1);
    img_extend(0,0) = img(0,0);
    img_extend(x_size+1,0) = img(x_size-1,0);
    img_extend(0,y_size+1) = img(0,y_size-1);
    img_extend(x_size+1,y_size+1) = img(x_size-1,y_size-1);
    for(int x = 0; x<x_size; ++x)
    {
        img_extend(x+1,0) = img(x, 0);
        img_extend(x+1, y_size+1) = img(x, y_size-1);
    }
    for(int y = 0; y<y_size; ++y)
    {
        img_extend(0,y+1) = img(0,y);
        img_extend(x_size+1,y+1) = img(x_size-1,y);
    }

    for(int x=1; x<=x_size; ++x)
    {
        for(int y=1; y<=y_size; ++y)
        {
            if(img_extend(x,y)>0)
            {
                int nx = x-1, ny = y-1;
                img(nx,ny)=1;
                if(nx>0) img(nx-1,ny)=1;
                if(ny>0) img(nx,ny-1)=1;
                if(nx+1<x_size) img(nx+1,ny)=1;
                if(ny+1<y_size) img(nx,ny+1)=1;
            }
        }
    }
}

void AncillaryMethods::MorphologyOpen(Matrix<double>& img)
{
    MorphologyErode(img);
    MorphologyDilate(img);
}


void intersectPlane(Vector<double>& gp, double gpd, Vector<double>& ray1, Vector<double>& ray2, Vector<double>& point)
{
    Vector<double> diffRay;
    diffRay = ray1;
    diffRay -= ray2;

    double den = DotProduct(gp, diffRay);
    double t = (DotProduct(gp, ray1) + gpd) / den;

    point = ray1;
    diffRay = (ray2);
    diffRay -= (ray1);
    diffRay *= t;
    point += diffRay;
}

void getRay(const Vector<double>& x, Vector<double>& ray1, Vector<double>& ray2, const Camera& cam)
{
    Matrix<double> rot = cam.get_R();
    Matrix<double> Kinv = cam.get_K();
    Vector<double> pos = cam.get_t();

    ray1 = pos;

    Kinv.inv();

    rot *= Kinv;
    ray2 = rot * x;
    ray2 += ray1;
}

Vector<double> AncillaryMethods::backprojectGP(const Vector<double> &pos2D, const Camera &cam, const Vector<double> &gp)
{
    Vector<double> gpN(3);
    gpN(0) = (gp(0));
    gpN(1) = (gp(1));
    gpN(2) = (gp(2));

    Vector<double> ray1;
    Vector<double> ray2;

    getRay(pos2D, ray1, ray2, cam);

    Vector<double> point3D;

    intersectPlane(gpN, gp(3), ray1, ray2, point3D);
    point3D *= Globals::WORLD_SCALE;

    return point3D;

}

int AncillaryMethods::getSizeIdx(Vector < FrameInlier >& idx) {
    int nrInlier = 0;

    for (int i = 0; i < idx.getSize(); i++) {
        nrInlier += idx(i).getNumberInlier();
    }

    return nrInlier;
}

void AncillaryMethods::intersectIdx(Vector< FrameInlier >& idx1, Vector< FrameInlier >& idx2, Vector< FrameInlier >& intersection) {

    int j = 0;
    int k = 0;
    intersection.clearContent();
    Vector<int> inter;
    Vector<int> inlier1;
    Vector<int> inlier2;
    Vector<double> weight1;
    Vector<double> weight2;

    if (idx1.getSize() > 0 && idx2.getSize() > 0) {
        for (int i = 0; i < idx1.getSize(); i++) {

            while (idx1(i).getFrame() > idx2(j).getFrame() && j < idx2.getSize() - 1) {
                j++;
            }

            if (j == idx2.getSize())
                break;

            if (idx1(i).getFrame() == idx2(j).getFrame()) {
                inlier1 = idx1(i).getInlier();
                inlier2 = idx2(j).getInlier();

                weight1 = idx1(i).getWeight();
                weight2 = idx2(j).getWeight();

                inlier1.intersection(inlier2, inter);

                if (inter.getSize() > 0) {
                    FrameInlier inlier(idx1(i).getFrame());

                    for ( int l = 0; l < inlier1.getSize(); l++) {
                        while (inlier1(l) > inlier2(k) && k < inlier2.getSize() - 1) {
                            k++;
                        }

                        if (k == inlier2.getSize())
                            break;

                        if (inlier1(l) == inlier2(k)) {
                            inlier.addInlier(inlier1(l));
                            inlier.addWeight(weight1(l));
                            inlier.addWeight(weight2(k));
                        }
                    }
                    k = 0;
                    intersection.pushBack(inlier);
                }
            }
        }
    }
}

Matrix<double> AncillaryMethods::smoothTrajMatrix(Matrix<double> &allPoints, int nSmoothSize)
{

    int width = floor(nSmoothSize/2.0);
    int nrX = allPoints.y_size();

    Matrix<double> partMat;
    Matrix<double> smoothedPos(allPoints.x_size(), allPoints.y_size());
    Vector<double> mean;

    // ***********************************************************************
    //   Smooth the position of the trajectory
    // ***********************************************************************

    for(int i = 0; i < nrX; i++)
    {
        int cw = min(min(i, width),nrX-i-1);
        int upperCol = i - cw;
        int lowerCol = i + cw;

        allPoints.cutPart(0, allPoints.x_size()-1, upperCol, lowerCol, partMat);
        partMat.sumAlongAxisX(mean);

        if((lowerCol - upperCol) > 0)
        {
            mean *=(1.0/((lowerCol - upperCol) + 1));
        }
        smoothedPos.insertRow(mean, i);
    }

    return smoothedPos;

}

void AncillaryMethods::swapVectorVolume(Vector<Volume <double> >& src)
{
    Vector<Volume <double> > aux;
    for( int i = 0; i < src.getSize(); i++)
    {
        aux.pushBack(src(i));
    }
    src.clearContent();
    for( int i = 0; i < aux.getSize(); i++)
    {
        src.pushBack(aux(aux.getSize()-1 - i));
    }
}

void AncillaryMethods::swapVectorMatrix(Vector<Matrix <double> >& src)
{
    Vector<Matrix <double> > aux;
    for( int i = 0; i < src.getSize(); i++)
    {
        aux.pushBack(src(i));
    }
    src.clearContent();
    for( int i = 0; i < aux.getSize(); i++)
    {
        src.pushBack(aux(aux.getSize() - 1 - i));
    }
}

void AncillaryMethods::compute_rectangle(Vector<double>& main4D, Vector<double>& ort4D, Vector<double>& Lmax, Vector<double>& x, Matrix<double>& result) {

    Vector<double> q(3);
    Vector<double> copyMain4D;
    copyMain4D = main4D;
    Vector<double> copyOrt4D;
    copyOrt4D = ort4D;

    // -----------------------------------------------
    // -----------compute q1 Start--------------------
    // -----------------------------------------------

    q = x;

    copyMain4D *= Lmax(0);
    copyOrt4D *= Lmax(1);
    q += copyMain4D;
    q += copyOrt4D;
    result.insertRow(q, 0);

    // -----------compute q2 Start--------------------

    q = x;
    q -= copyMain4D;
    q += copyOrt4D;
    result.insertRow(q, 3);

    // -----------compute q3 Start--------------------

    q = x;
    q += copyMain4D;
    q -= copyOrt4D;
    result.insertRow(q, 1);

    // -----------compute q4 Start--------------------

    q = x;
    q -= copyMain4D;
    q -= copyOrt4D;
    result.insertRow(q, 2);

}



Matrix<double> AncillaryMethods::conv1D(Matrix<double>& im, Vector<double>& kernel, bool dirFlag)
{

    // dirFalg == true --> in x dir,
    // dirFalg == false --> in y dir
    int kernelSize = kernel.getSize();
    int kCenter = floor(kernelSize/2.0);

    int nn;

    Matrix<double> result(im.x_size(), im.y_size(), 0.0);

    if(!dirFlag)
    {
        for(int i = 0; i < im.x_size(); i++)
        {
            for(int j = 0; j < im.y_size(); j++)
            {
                for (int n = 0; n < kernelSize; n++)
                {
                    nn = kernelSize - 1 - n;
                    int posx = i;
                    int posy = j + (n - kCenter);
                    if(posy >= 0 && posy < im.y_size())
                    {
                        result(i,j) += max(0.0, im(posx, posy))*kernel(nn);
                    }
                }
            }
        }
    }
    else
    {
        for(int i = 0; i < im.x_size(); i++)
        {
            for(int j = 0; j < im.y_size(); j++)
            {
                for (int n = 0; n < kernelSize; n++)
                {
                    nn = kernelSize - 1 - n;
                    int posx = i + (n - kCenter);
                    int posy = j;
                    if(posx >= 0 && posx < im.x_size())
                    {
                        result(i,j) += max(0.0, im(posx, posy))*kernel(nn);
                    }
                }
            }
        }
    }

    return result;
}


void AncillaryMethods::ExtractSlopsOnBorder(const Matrix<double>& image, Vector<double>& ys, Vector<double>& slopes, int start_x, int end_x, int start_y, int end_y)
{
    if(end_x<0)
        end_x = image.x_size() - start_x - 1;
    if(end_y<0)
        end_y = image.y_size() - start_y - 1;

    slopes.setSize(end_x - start_x + 1);

    ys.setSize(end_x - start_x + 1);

    for(int x = start_x ; x <= end_x; ++x)
    {
        int y0= end_y;
        for(int y=start_y; y<=end_y; ++y)
        {
            if(image(x,y) > 0)
            {
                y0 = y;
                break;
            }
        }
        ys(x - start_x) = end_y - y0;
    }

    //    Vector<double> ys = ys;
    ys = AncillaryMethods::conv1D(ys, AncillaryMethods::getGaussian1D(3,1));

    for(int i=0; i<ys.getSize()-1; ++i)
        slopes(i) = ys(i+1) - ys(i);
    slopes(end_x - start_x) = 0;
}

Vector<int> AncillaryMethods::FindLocalMax(Vector<double>& slopes)
{
    Vector<int> xs;
    xs.reserveCapacity(slopes.getSize());

    int last_slope_sign = 0, last_x = 0;

    for(int i=0;i<slopes.getSize();++i)
        slopes(i) = round(slopes(i));

    if(slopes(0)<0)
    {
        xs.pushBack(0);
        last_x = 0;
        last_slope_sign = -1;
    }

    for(int x = 1; x<slopes.getSize(); ++x)
    {
        double current_slope = slopes(x);
        switch(last_slope_sign)
        {
        case 0:
            if(current_slope>0)
            {
                last_x=x;
                last_slope_sign = 1;
            }
            else if(current_slope<0)
            {
                last_x=x;
                last_slope_sign = -1;
            }
            break;
        case 1:
            if(current_slope>0)
            {
                last_x=x;
            }
            else if(current_slope<0)
            {
                xs.pushBack((int)(ceil((x+last_x)/2.0)));
                last_x=x;
                last_slope_sign = -1;
            }
            break;
        case -1:
            if(current_slope>0)
            {
                last_x=x;
                last_slope_sign = 1;
            }
            else if(current_slope<0)
            {
                last_x=x;
            }
            break;
        }
    }

    return xs;
}

void AncillaryMethods::ExtractBorder(const Matrix<double>& image, Vector<double>& ys, int start_x, int end_x, int start_y, int end_y)
{
    if(end_x<0)
        end_x = image.x_size() - start_x - 1;
    if(end_y<0)
        end_y = image.y_size() - start_y - 1;

    ys.setSize(end_x - start_x + 1);

    for(int x = start_x ; x <= end_x; ++x)
    {
        int y0= end_y;
        for(int y=start_y; y<=end_y; ++y)
        {
            if(image(x,y) > 0)
            {
                y0 = y;
                break;
            }
        }
        ys(x - start_x) = end_y - y0;
    }

    //    ys = AncillaryMethods::conv1D(ys, AncillaryMethods::getGaussian1D(3,1));
}
