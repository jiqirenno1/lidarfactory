//
// Created by ubuntu on 2020/11/24.
//
#include <opencv2/opencv.hpp>
#include <iostream>

void DrawCircle(cv::Mat img, cv::Point center, std::string text, cv::Scalar bgr)
{
    int thickness = -1;
    int lineType = 8;
    cv::circle(img, center, 15, bgr, thickness, lineType);
    cv::putText(img, text, cv::Point(center.x-4, center.y-3), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(255,0,0));
}


bool Isbound(int x_) {
    if(x_<50||x_>550)
    {
        return true;
    }
    else
    {
        return false;
    }
}
std::string Iscrush(std::vector<cv::Point> ps)
{
    std::map<int, std::string> map{{0, "person1"},{1, "person2"},{2, "car1"},{3, "car2"}};

    std::string out="";
    for(int i=0;i<ps.size();i++)
    {
        for(int j=i+1;j<ps.size();j++)
        {
            cv::Point pi = ps[i];
            cv::Point pj = ps[j];
            if(sqrt((pj.x-pi.x)*(pj.x-pi.x)+(pj.y-pi.y)*(pj.y-pi.y))<5){
                out += map[i]+" crushed "+map[j]+"  ";
            }

        }
    }
    return out;


}

int main()
{
    int sx1=1, sx2=1, sx3=1, sx4=1,sy1=1, sy2=1, sy3=1, sy4=1;
    int px1=50,px2=50,px3=50,px4=50,py1=50,py2=50,py3=50,py4=50;
    std::vector<cv::Point> ps;
    std::string text1="";
    int i= 1000;
    while(1)
    {
        ps.clear();
        cv::Mat im = cv::Mat(600,600, CV_8UC3, cv::Scalar(0,0,0));
        cv::rectangle(im, cv::Point(50,50), cvPoint(550,550), cv::Scalar(255,255,255), -1);

      cv::Point p1(px1,py1+100);
      ps.push_back(p1);

      if(Isbound(p1.x))
      {
          sx1=-sx1;
      }
      px1 = px1+sx1;

        if(Isbound(p1.y))
        {
            sy1=-sy1;
        }
        py1 = py1+sy1;


        cv::Point p2(px2,300);
        ps.push_back(p2);

        if(Isbound(p2.x))
        {
            sx2=-sx2;
        }
        px2 = px2+sx2;


        cv::Point p3(px3+50,py3);
        ps.push_back(p3);

        if(Isbound(p3.x))
        {
            sx3=-sx3;
        }
        px3 = px3+sx3;

        if(Isbound(p3.y))
        {
            sy3=-sy3;
        }
        py3 = py3+sy3;

        cv::Point p4(300, py4);
        ps.push_back(p4);


        if(Isbound(p4.y))
        {
            sy4=-sy4;
        }
        py4 = py4+sy4;
        std::string text0 = Iscrush(ps);
        if(!text0.empty())
        {
            text1 = text0;
        }

        DrawCircle(im, p1, "person1",cv::Scalar(0,255,255));
        DrawCircle(im, p2, "person2",cv::Scalar(0,255,255));
        DrawCircle(im, p3, "car1",cv::Scalar(0,255,0));
        DrawCircle(im, p4, "car2",cv::Scalar(0,255,0));
        cv::putText(im, text1, cv::Point(50, 550), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0,0,255));
        cv::imshow("1", im);
        cv::waitKey(10);


    }
}
