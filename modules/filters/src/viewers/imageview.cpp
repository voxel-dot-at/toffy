#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

#include <boost/log/trivial.hpp>
#include <boost/any.hpp>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "toffy/viewers/imageview.hpp"

using namespace toffy;
using namespace cv;
using namespace std;

std::size_t ImageView::_filter_counter = 1;
const std::string ImageView::id_name = "imageview";

ImageView::ImageView(): Filter(ImageView::id_name,_filter_counter),
    _scale(1.f), _max(-1), _min(-1), _in_img("img"),
    _gray(true), _enabled(true)
{
    _filter_counter++;
}

ImageView::~ImageView(){
    cv::destroyWindow(name() + " " + _in_img);
}

void ImageView::updateConfig(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _scale = pt.get<double>("options.scale",_scale);
    _max = pt.get<double>("options.max",_max);
    _min = pt.get<double>("options.min",_min);
    _gray = pt.get<bool>("options.gray",_gray);
    _enabled = pt.get<bool>("options.enabled",_enabled);

    _in_img = pt.get<string>("inputs.img",_in_img);

}

boost::property_tree::ptree ImageView::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.scale", _scale);
    pt.put("options.max", _max);
    pt.put("options.min", _min);
    pt.put("options.gray", _gray);
    pt.put("options.enabled", _enabled);
    pt.put("inputs.img", _in_img);

    return pt;
}

bool ImageView::filter(const Frame &in, Frame& out)
{
    using namespace boost::posix_time;

    ptime start = microsec_clock::local_time();
    boost::posix_time::time_duration diff;

    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    if (!_enabled)
        return true; // skip it - performance tests.

    boost::shared_ptr<cv::Mat> img;
    try {
        img = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData(_in_img));
    } catch(const boost::bad_any_cast &) {
        BOOST_LOG_TRIVIAL(warning) << "Could not cast input " << _in_img <<
                                      ", filter  " << id() <<" not applied.";
        return false;
    }

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "ImageView get: " << diff.total_microseconds();

    show = *img;
    if (!show.data) {
        BOOST_LOG_TRIVIAL(warning) << "missing image " << _in_img <<
                                      ", filter  " << id() <<" not applied.";
        return false;
    }

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "ImageView got: " << diff.total_microseconds();

    //show.convertTo(show,CV_8U,255.0/(1000-0),-255.0*0/(1000-0));
    if (_gray) {
        if (_max == _min) {
            minMaxIdx(show, &_min, &_max);
        }
        show.convertTo(show,CV_8U,255.0/(_max-_min),-255.0*_min/(_max-_min));

        diff = boost::posix_time::microsec_clock::local_time() - start;
        BOOST_LOG_TRIVIAL(debug) << "ImageView gray2rgb: " << diff.total_microseconds();
    }

    if( _scale != 1.f ) {
        int method = _scale < 1 ? INTER_AREA : INTER_LINEAR;
        resize(show, show, Size(), _scale, _scale, method);

        diff = boost::posix_time::microsec_clock::local_time() - start;
        BOOST_LOG_TRIVIAL(debug) << "ImageView resize: " << diff.total_microseconds();
    }

    cv::namedWindow(name() + " " + _in_img, cv::WINDOW_NORMAL);
    imshow(name() + " " + _in_img, show);

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "ImageView fin: " << diff.total_microseconds();

    return true;
}

