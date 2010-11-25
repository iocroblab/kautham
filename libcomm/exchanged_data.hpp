

#ifndef IOC_COMM_DATA_HPP
#define IOC_COMM_DATA_HPP

#include <string>
#include <vector>

namespace ioc_comm {
  enum DEVICE{ NONE, HAPTIC, TXROB1,TXROB2, DRLHAND, GLOVE };
  enum DATATYPE{UNDEF=0, SETTING=5, CARTPOS=10, CARTVEL=20, CARTFORCE=30,
    ARTIPOS=50, ARTIVEL=100, ARTIFORCE=150};
    struct baseData{
        DATATYPE id;
        std::string time_stamp;
        std::vector<double> _data;

        baseData(){
          id = UNDEF;
          time_stamp="00000.000";
        }

        template <typename Archive>
        void serialize(Archive& ar, const unsigned int version)   {
            ar & id;
            ar & time_stamp;
            ar & _data;
        }
    };

    struct settings:baseData {
      settings (){ id=SETTING; }
    };

	  namespace cartesian {
	  // Structure to hold information about a cartesian position.
      struct position:baseData {
        position (){ id=CARTPOS; _data.resize(6);}
		  };

      struct velocity:baseData {
        velocity (){ id=CARTVEL; _data.resize(6); }
		  };

      struct force:baseData {
        force (){id=CARTFORCE; _data.resize(6);}
		  };
	  } // namespace cartesian

	  namespace articular {

		/// Structure to hold information about a articular position.
      struct position:baseData {
        position(unsigned int dim){ id=ARTIPOS; _data.resize(dim); }
		  };

		/// Structure to hold information about a articular position.
		  struct velocity:baseData {
        velocity(unsigned int dim){ id=ARTIVEL; _data.resize(dim);  }
		  };

		/// Structure to hold information about a articular position.
		  struct force:baseData {
        force(unsigned int dim){ id=ARTIFORCE; _data.resize(dim);  }
      };
	} // namespace articular

  //! This function format the time stamp to send to the client.
  static std::string cal_time_stamp(){
      using namespace boost::posix_time; /* put that in functions where you work with time (namespace) */

      ptime now = microsec_clock::local_time(); // current *LOCAL TIMEZONE* time/date
      time_duration tod = now.time_of_day();

      std::ostringstream strTime;
      int sec =tod.seconds() + tod.minutes()*60 + tod.hours()*3600;
      int mlsec = tod.fractional_seconds() / 1000;

      strTime << std::setfill('0') << std::setw(5) << sec;
      strTime << '.' ;
      strTime << std::setfill('0') << std::setw(3) << mlsec;

      return strTime.str();
  }

  typedef std::vector<baseData> vecData;
  typedef boost::shared_ptr<vecData> vecData_ptr;

} // namespace ioc_comm

#endif // IOC_COMM_DATA_HPP
