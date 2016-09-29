/**Code for timestamping**/
#include <iostream>
#include <fstream>
#include <stdio.h>      // for sprintf()
#include <iostream>     // for console output
#include <string>       // for std::string
#include <boost/date_time/posix_time/posix_time.hpp>


#define LOGNAME_FORMAT "/home/debian/log/"
#define LOGNAME_SIZE 20

FILE *logfile(void)
{
    static char name[LOGNAME_SIZE];
    time_t now = now_str();
    strftime(name, sizeof(name), LOGNAME_FORMAT, localtime(&now));
    return fopen(name, "ab");
}
//-----------------------------------------------------------------------------
// Format current time (calculated as an offset in current day) in this form:
//
//     "hh:mm:ss.SSS" (where "SSS" are milliseconds)
//-----------------------------------------------------------------------------
std::string now_str()
{
    // Get current time from the clock, using microseconds resolution
    const boost::posix_time::ptime now =
        boost::posix_time::microsec_clock::local_time();

    // Get the time offset in current day
    const boost::posix_time::time_duration td = now.time_of_day();

    //
    // Extract hours, minutes, seconds and milliseconds.
    //
    // Since there is no direct accessor ".milliseconds()",
    // milliseconds are computed _by difference_ between total milliseconds
    // (for which there is an accessor), and the hours/minutes/seconds
    // values previously fetched.
    //
    const long hours        = td.hours();
    const long minutes      = td.minutes();
    const long seconds      = td.seconds();
    const long milliseconds = td.total_milliseconds() -
                              ((hours * 3600 + minutes * 60 + seconds) * 1000);

    //
    // Format like this:
    //
    //      hh:mm:ss.SSS
    //
    // e.g. 02:15:40:321
    //
    //      ^          ^
    //      |          |
    //      123456789*12
    //      ---------10-     --> 12 chars + \0 --> 13 chars should suffice
    //
    //
    char buf[40];
    sprintf(buf, "%02ld:%02ld:%02ld.%03ld",
        hours, minutes, seconds, milliseconds);

    return buf;
}

int main()
{
        FILE *file = logfile();
        std::ofstream csv_file;
      csv_file.open(now_str+"logstart.csv");
      while(1){
        csv_file << "Timestamp, Front, Back, Left, Right\n";
        csv_file << now_str()+","+get_sensor("front")+","+get_sensor("back")+","+get_sensor("left")+","+get_sensor("right")+"\n";
      }
      csv_file.close();
}