#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <communication/multi_socket.h>
#include <models/tronis/ImageFrame.h>
#include <grabber/opencv_tools.hpp>

#include <geometry/coord_sys_adapter_tronis_model.h>
#include <models/tronis/BoxData.h>
#include <cmath>

using namespace std;

class LaneAssistant
{
    // insert your custom functions and algorithms here
public:
    double steering_beforePID = 0;
    double p_Errorsteering = 0;
    double i_Errorsteering = 0;

    double steeringdouble = 0;
    double throttledouble ;
    string throttle = "0.75";
    string combined;
    double distance;

	double errorthrottle = 0;
    double prevErrorthrottle = 0;

	double errorabstand = 0;
    double prevErrorabstand = 0;
    string strict1 = "maximal Geschwindigkeit:";
    string strict2 = "Sicher Abstand:";
    string strict=strict1;

	double abs_steering;
	double max_speed=75;
    int soll_distance = 50;
    double soll_speed=75;

    LaneAssistant()
    {
    }

    bool processData( tronis::CircularMultiQueuedSocket& socket )
    {
        
        steering_beforePID = detectLanes();
		
        double pTerm = 0.5 * steering_beforePID;
        double dTerm = 0.001 * ( ( steering_beforePID - p_Errorsteering ) / 0.1 );
        i_Errorsteering += steering_beforePID * 0.00000002;
        double iTerm = 0.12 * i_Errorsteering;
        p_Errorsteering = steering_beforePID;
        steeringdouble = pTerm + dTerm+iTerm;
        string steering = std::to_string( steeringdouble );
        distance = sqrt( pos_x * pos_x + pos_y * pos_y );
        abs_steering = std::abs( steeringdouble );
        soll_speed = max_speed - abs_steering * 125;
        soll_distance = static_cast<int>( ego_velocity_ * 0.036 );
        
        
        if( distance > 5500 )
        {
            errorthrottle = ( soll_speed - ego_velocity_ * 0.036 ) / soll_speed + 0.71;

            double pTermthrottle=0.9*errorthrottle;
            double dTermthrottle = 0.1 * ((errorthrottle-prevErrorthrottle)/0.1);
            prevErrorthrottle = errorthrottle;
            throttledouble = pTermthrottle+dTermthrottle;
            strict = strict1 + std::to_string( soll_speed );
        }
        else
        {


            throttledouble = ( distance - 5000 ) / 5000 + 0.71 +
                             0.1 * ( ( distance - 5000 - prevErrorabstand ) / 0.1 ) - 40 / distance;
            prevErrorabstand = distance - 5000;
            strict = strict2 + std::to_string( soll_distance ) + "m";
        }

        throttle = std::to_string( throttledouble );
        combined = steering + "+" + throttle;
        socket.send( tronis::SocketData( combined ) );
        // do stuff with data
        // send results via socket
        return true;
    }

protected:
    std::string image_name_;
    cv::Mat image_;
    tronis::LocationSub ego_location_;
    tronis::OrientationSub ego_orientation_;
    double ego_velocity_;

	
    tronis::ObjectSub object_;
    tronis::LocationSub otherlocation_;

	float pos_x;
    float pos_y;

	float ego_pos_x;
    float ego_pos_y;
	


    // Function to detect lanes based on camera image
    // Insert your algorithm here
    double detectLanes()
    {
        int image_width = image_.cols;
        int image_height = image_.rows;
        cv::Mat cannyedimage_;
        cv::Canny( image_, cannyedimage_, 100, 200 );

        cv::Mat mask = cv::Mat::zeros( image_.size(), CV_8UC1 );
        std::vector<cv::Point> region_of_interest_vertices = {
            cv::Point( 0, image_height * 0.85 ), cv::Point( image_width * 0.2, image_height * 0.6 ),
            cv::Point( image_width * 0.8, image_height * 0.6 ),
            cv::Point( image_width, image_height * 0.85 )};

        std::vector<std::vector<cv::Point>> pts{region_of_interest_vertices};
        cv::Scalar match_mask_color = cv::Scalar( 255, 255, 255 );
        cv::fillPoly( mask, pts, match_mask_color );
        cv::Mat masked_image;
        cv::bitwise_and( cannyedimage_, mask, masked_image );

        std::vector<cv::Vec4i> lines;
      
        cv::HoughLinesP( masked_image, lines, 6, CV_PI / 180, 70, 40, 40 );
        cv::Mat blended_img;

        std::vector<int> left_line_x, left_line_y, right_line_x, right_line_y;
        for( auto line : lines )
        {
            double slope = static_cast<double>( line[3] - line[1] ) / ( line[2] - line[0] );
            if( std::fabs( slope ) < 0.3 )
            {
                continue;
            }
            if( slope <= 0 )
            {
                left_line_x.push_back( line[0] );
                left_line_x.push_back( line[2] );
                left_line_y.push_back( line[1] );
                left_line_y.push_back( line[3] );
                
            }
            else
            {
                right_line_x.push_back( line[0] );
                right_line_x.push_back( line[2] );
                right_line_y.push_back( line[1] );
                right_line_y.push_back( line[3] );
                
            }
        }

        double min_y = ( image_height * ( 3.0 / 5 ) );
        double max_y = ( image_height );

        // left
        int left_n = left_line_x.size();
        double sumleftX = 0.0, sumleftY = 0.0, sumleftXY = 0.0, sumleftXX = 0.0;
        for( int i = 0; i < left_n; ++i )
        {
            sumleftX += left_line_x[i];
            sumleftY += left_line_y[i];
            sumleftXY += left_line_x[i] * left_line_y[i];
            sumleftXX += left_line_x[i] * left_line_x[i];
        }
        double left_m = ( left_n * sumleftXY - sumleftX * sumleftY ) /
                        ( left_n * sumleftXX - sumleftX * sumleftX ); //(N*sumXY-sumX*sumY)/(N*sumXX-sumX*sumX)
        double left_c = ( sumleftY - left_m * sumleftX ) / left_n;

        double left_x_start =  ( max_y - left_c ) / left_m ;
        double left_x_end =  ( min_y - left_c ) / left_m;

        // right
        int right_n = right_line_x.size();
        double sumrightX = 0.0, sumrightY = 0.0, sumrightXY = 0.0, sumrightXX = 0.0;
        for( int i = 0; i < right_n; ++i )
        {
            sumrightX += right_line_x[i];
            sumrightY += right_line_y[i];
            sumrightXY += right_line_x[i] * right_line_y[i];
            sumrightXX += right_line_x[i] * right_line_x[i];
        }
        double right_m = ( right_n * sumrightXY - sumrightX * sumrightY ) /
                         ( right_n * sumrightXX - sumrightX * sumrightX );
        double right_c = ( sumrightY - right_m * sumrightX ) / right_n;

        double right_x_start =( max_y - right_c ) / right_m ;
        double right_x_end =  ( min_y - right_c ) / right_m ;

        cv::Scalar color1 = cv::Scalar( 0, 0, 255 );
        cv::Scalar color2 = cv::Scalar( 0, 255, 255 );
        cv::Mat line_img = cv::Mat::zeros( image_.size(), CV_8UC3 );

       cv::line( image_, cv::Point( left_x_start, max_y ), cv::Point( left_x_end, min_y ), color1,
                  3, cv::LINE_AA );
        cv::line( image_, cv::Point( right_x_start, max_y ), cv::Point( right_x_end, min_y ),
                  color2, 3, cv::LINE_AA );



        
         // do stuff
        if(sumleftX==0&&sumrightX==0)
		{
            return 0;
		}
		else
		{
                    if(sumleftX==0)
                    {
                        return -( max_y - min_y )/(0.001+(right_x_start-right_x_end));
                    }
					else if (sumrightX==0)
					{
                       return  -( max_y - min_y ) / (0.001+( left_x_start - left_x_end ));
					}
                    else
				    {
                       return -( max_y - min_y ) /(0.001+ ( right_x_start - right_x_end )) -
                            ( max_y - min_y ) / (0.001+( left_x_start - left_x_end ));
					}
		}


        
    }

    bool processPoseVelocity( tronis::PoseVelocitySub* msg )
    {
        ego_location_ = msg->Location;
        ego_orientation_ = msg->Orientation;
        ego_velocity_ = msg->Velocity;
        ego_pos_x = ego_location_.X;
        ego_pos_y = ego_location_.Y;
        return true;
    }
	
    bool processObject( tronis::BoxDataSub* sensorData)
    {

        if( sensorData->Objects.size() != 0 )
        {
        for( size_t i = 0; i < sensorData->Objects.size();i++ )
			{
            
		
            object_ = sensorData->Objects[i];
            cout << object_.ActorName.Value() << endl;
            if( object_.ActorName.Value() == "GenericCoupe_2" )
				{
            otherlocation_ = object_.Pose.Location;
            pos_x = otherlocation_.X;
            pos_y = otherlocation_.Y;
                }
			else
			{
                            pos_x = 7071060;
                            pos_y = 7071060;
			}
            }
		}
        else
        {
          pos_x = 7071060;
          pos_y = 7071060;
        }
		// do stuff
        return true;
    }

    // Helper functions, no changes needed
public:
    // Function to process received tronis data
    bool getData( tronis::ModelDataWrapper data_model )
    {
        if( data_model->GetModelType() == tronis::ModelType::Tronis )
        {
            std::cout << "Id: " << data_model->GetTypeId() << ", Name: " << data_model->GetName()
                      << ", Time: " << data_model->GetTime() << std::endl;

            // if data is sensor output, process data
            switch( static_cast<tronis::TronisDataType>( data_model->GetDataTypeId() ) )
            {
                case tronis::TronisDataType::Image:
                {
                    processImage( data_model->GetName(),
                                  data_model.get_typed<tronis::ImageSub>()->Image );
                    break;
                }
                case tronis::TronisDataType::ImageFrame:
                {
                    const tronis::ImageFrame& frames(
                        data_model.get_typed<tronis::ImageFrameSub>()->Images );
                    for( size_t i = 0; i != frames.numImages(); ++i )
                    {
                        std::ostringstream os;
                        os << data_model->GetName() << "_" << i + 1;

                        processImage( os.str(), frames.image( i ) );
                    }
                    break;
                }
                case tronis::TronisDataType::ImageFramePose:
                {
                    const tronis::ImageFrame& frames(
                        data_model.get_typed<tronis::ImageFramePoseSub>()->Images );
                    for( size_t i = 0; i != frames.numImages(); ++i )
                    {
                        std::ostringstream os;
                        os << data_model->GetName() << "_" << i + 1;

                        processImage( os.str(), frames.image( i ) );
                    }
                    break;
                }
                case tronis::TronisDataType::PoseVelocity:
                {
                    processPoseVelocity( data_model.get_typed<tronis::PoseVelocitySub>() );
                    break;
                }
                case tronis::TronisDataType::BoxData:
                {
                    processObject( data_model.get_typed<tronis::BoxDataSub>() );
                    break;
                }
                default:
                {
                    std::cout << data_model->ToString() << std::endl;
                    break;
                }
            }
            return true;
        }
        else
        {
            std::cout << data_model->ToString() << std::endl;
            return false;
        }
    }

protected:
    // Function to show an openCV image in a separate window
    void showImage( std::string image_name, cv::Mat image, double velocity )
    {
        cv::Mat out = image;
        if( image.type() == CV_32F || image.type() == CV_64F )
        {
            cv::normalize( image, out, 0.0, 1.0, cv::NORM_MINMAX, image.type() );
        }
        int velocityint = static_cast<int>( velocity*0.036 );
        int abstandint = static_cast<int>( distance * 0.01 );
        if(abstandint>10000)
        {
            abstandint = -1;
        }
        int steeringint = static_cast<int>( steeringdouble*100 );
        string velocitystr ="Geschwindigkeit: "+ std::to_string( velocityint )+"km/h";
        string abstandstr = "Abstand: "+std::to_string( abstandint )+"m";
        string steeringstr = "Lenkung: " + std::to_string( steeringint * 0.01 );
        string throttlestr = "throttle: " + throttle;

        cv::Point textPosition1( 10, 30 );
        cv::Point textPosition2( 10, 55 );
        cv::Point textPosition3( 10, 80 );
        cv::Point textPosition4( 10, 105 );
        cv::Point textPosition5( 10, 130 );
        
        
        cv::putText( image, velocitystr, textPosition1, cv::FONT_HERSHEY_SIMPLEX, 1, ( 255, 0, 0 ) ,2);
        cv::putText( image, abstandstr, textPosition2, cv::FONT_HERSHEY_SIMPLEX, 1, ( 255, 0, 0 ),
                     2 );
        cv::putText( image, steeringstr, textPosition3, cv::FONT_HERSHEY_SIMPLEX, 1, ( 255, 0, 0 ),
                     2 );
        cv::putText( image, throttlestr, textPosition4, cv::FONT_HERSHEY_SIMPLEX, 1, ( 255, 0, 0 ),
                     2 );
        cv::putText( image, strict, textPosition5, cv::FONT_HERSHEY_SIMPLEX, 1, ( 255, 0, 0 ), 2 );
        
        
        cv::namedWindow( image_name.c_str(), cv::WINDOW_NORMAL );
        cv::imshow( image_name.c_str(), out );
    }

    // Function to convert tronis image to openCV image
    bool processImage( const std::string& base_name, const tronis::Image& image )
    {
        std::cout << "processImage" << std::endl;
        if( image.empty() )
        {
            std::cout << "empty image" << std::endl;
            return false;
        }

        image_name_ = base_name;
        image_ = tronis::image2Mat( image );

        detectLanes();
        showImage( image_name_, image_ ,ego_velocity_);

        return true;
    }
};

// main loop opens socket and listens for incoming data
int main( int argc, char** argv )
{
    std::cout << "Welcome to lane assistant" << std::endl;

    // specify socket parameters
    std::string socket_type = "TcpSocket";
    std::string socket_ip = "127.0.0.1";
    std::string socket_port = "7778";

    std::ostringstream socket_params;
    socket_params << "{Socket:\"" << socket_type << "\", IpBind:\"" << socket_ip
                  << "\", PortBind:" << socket_port << "}";

    int key_press = 0;  // close app on key press 'q'
    tronis::CircularMultiQueuedSocket msg_grabber;
    uint32_t timeout_ms = 500;  // close grabber, if last received msg is older than this param

    LaneAssistant lane_assistant;

    while( key_press != 'q' )
    {
        std::cout << "Wait for connection..." << std::endl;
        msg_grabber.open_str( socket_params.str() );

        if( !msg_grabber.isOpen() )
        {
            printf( "Failed to open grabber, retry...!\n" );
            continue;
        }

        std::cout << "Start grabbing" << std::endl;
        tronis::SocketData received_data;
        uint32_t time_ms = 0;

        while( key_press != 'q' )
        {
            // wait for data, close after timeout_ms without new data
            if( msg_grabber.tryPop( received_data, true ) )
            {
                // data received! reset timer
                time_ms = 0;

                // convert socket data to tronis model data
                tronis::SocketDataStream data_stream( received_data );
                tronis::ModelDataWrapper data_model(
                    tronis::Models::Create( data_stream, tronis::MessageFormat::raw ) );
                if( !data_model.is_valid() )
                {
                    std::cout << "received invalid data, continue..." << std::endl;
                    continue;
                }
                // identify data type
                lane_assistant.getData( data_model );
                lane_assistant.processData( msg_grabber );
            }
            else
            {
                // no data received, update timer
                ++time_ms;
                if( time_ms > timeout_ms )
                {
                    std::cout << "Timeout, no data" << std::endl;
                    msg_grabber.close();
                    break;
                }
                else
                {
                    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
                    key_press = cv::waitKey( 1 );
                }
            }
        }
        msg_grabber.close();
    }
    return 0;
}
