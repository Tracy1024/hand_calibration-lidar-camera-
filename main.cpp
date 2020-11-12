#include <iostream>
#include <stdio.h>

#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/console/parse.h>
#include <pcl-1.8/pcl/filters/passthrough.h>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/sample_consensus/model_types.h>
#include <pcl-1.8/pcl/features/board.h>

//get the path of image and point cloud
std::string path_img = "/home/yang/Documents/Ka-Raceing/as/imgs/frame0011.jpg";
std::string path_pointcloud = "/home/yang/Documents/Ka-Raceing/as/pcl/1603117156.516590000.pcd";

//initialization image processing
IplImage* src = 0;
IplImage* dst = 0;
std::vector<cv::Point2f> keypoints;

void imageSelect(int event, int x, int y, int flags, void* ustc)
{
    static CvPoint pre_pt = { -1,-1 };
    static CvPoint cur_pt = { -1,-1 };
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);
    char temp[16];

    if (event == CV_EVENT_LBUTTONDOWN)
    {
    cvCopy(dst,src);
    sprintf(temp, "(%d,%d)", x, y);
    pre_pt = cvPoint(x, y);
    cvPutText(src, temp, pre_pt, &font, cvScalar(0, 0, 0, 255));
    cvCircle(src, pre_pt, 3, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
    cvShowImage("src", src);
    cvCopy(src, dst);
    keypoints.push_back(cv::Point2f(x, y));
    }
    else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))
    {
    cvCopy(dst, src);
    sprintf(temp, "(%d,%d)", x, y);
    cur_pt = cvPoint(x, y);
    cvPutText(src, temp, cur_pt, &font, cvScalar(0, 0, 0, 255));
    cvShowImage("src", src);
    }
    else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))
    {
    cvCopy(dst, src);
    sprintf(temp, "(%d,%d)", x, y);
    cur_pt = cvPoint(x, y);
    cvPutText(src, temp, cur_pt, &font, cvScalar(0, 0, 0, 255));
    cvRectangle(src, pre_pt, cur_pt, cvScalar(0, 255, 0, 0), 1, 8, 0);
    cvShowImage("src", src);
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
    sprintf(temp, "(%d,%d)", x, y);
    cur_pt = cvPoint(x, y);
    cvPutText(src, temp, cur_pt, &font, cvScalar(0, 0, 0, 255));
    cvCircle(src, cur_pt, 3, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
    cvRectangle(src, pre_pt, cur_pt, cvScalar(0, 255, 0, 0), 1, 8, 0);
    cvShowImage("src", src);
    cvCopy(src, dst);
    }
}

// initialization point cloud processing
// structure used to pass arguments to the callback function
struct callback_args{
    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pointcloudSelect(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
        return;
    pcl::PointXYZ current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);

    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main()
{
    src = cvLoadImage((char*)path_img.data(), 1);
    dst = cvCloneImage(src);
    cvNamedWindow("src", 1);
    cvSetMouseCallback("src", imageSelect, 0);
    cvShowImage("src", src);
    cvWaitKey(0);

    cvDestroyAllWindows();

    cvReleaseImage(&src);
    cvReleaseImage(&dst);
    std::cout << "keypoints in image:    " << keypoints << std::endl;

    //cloud initialization
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredZ (new pcl::PointCloud<pcl::PointXYZ>);

    //read pcd file as point cloud
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ> (path_pointcloud, *cloud);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    // filter in Z direction(ground filter)
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.4, 2.0);
    pass.filter (*cloud_filteredZ);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    // Display pointcloud:
    viewer->addPointCloud(cloud_filteredZ, "pointcloud");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new  pcl::PointCloud<pcl::PointXYZ>);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pointcloudSelect, (void*)&cb_args);

    std::cout << "Shift+click on keypoints, then press 'Q'..." << std::endl;

    // Spin until 'Q' is pessed:
    viewer->spin();
    std::cout << "done." << std::endl;
    std::vector<cv::Point3f> keypoints3D;
    for(auto &point : clicked_points_3d->points)
        keypoints3D.push_back(cv::Point3f(point.x, point.y, point.z));

    cv::Mat rvecs, tvecs, inliers;

    //camera in-parameters
    cv::Mat parameter_cameraIn = (cv::Mat_<float>(3, 3) << 1.0438852850007693e+03, 0., 9.8201744178120418e+02, 0.,
            1.0419305182081262e+03, 5.9751257077062110e+02, 0., 0., 1.);
    cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) << -6.7620369858691673e-02, 1.5155999662397437e-01,
            -1.9355857052822513e-03, -1.0998916215830597e-03,
            -8.8335119665156961e-02);

    //calculate ex-parameter
    cv::solvePnPRansac(keypoints3D, keypoints, parameter_cameraIn, distCoeffs, rvecs, tvecs, false, 100, 1.0, 0.8, inliers);
    cv::Mat rMat;
    cv::Rodrigues(rvecs, rMat);

    std::cout << "===============Result=================" << std::endl;
    std::cout << "R_mat: " << rMat << std::endl;
    std::cout << "t: " << tvecs << std::endl;

    return 0;

}