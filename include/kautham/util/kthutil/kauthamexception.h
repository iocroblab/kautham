/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Nestor Garcia Hidalgo */


#include <exception>
#include <string>


namespace Kautham {


/*!
 * \brief The KthExcp class implements exceptions inside Kautham
 */
  class KthExcp:public std::exception {
  public:
      /*!
       * \brief KthExcp
       * \param message explains what happened
       * \param details gives details about what happened
       */
      KthExcp(std::string message = "An error ocurred", std::string details = "");


      ~KthExcp() throw();

      /*!
       * \brief what
       * \return what happened
       */
      char const* what() const throw();

      /*!
       * \brief more
       * \return details about what happened
       */
      char const* more() const throw();

  private:
      /*!
       * \brief msg stores what happened
       */
      std::string msg;

      /*!
       * \brief det stores details about what happened
       */
      std::string det;
  };
}

