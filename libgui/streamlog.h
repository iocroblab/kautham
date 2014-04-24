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

/* Author: Alexander Perez, Jan Rosell */
#ifndef STREAMLOG_H
#define STREAMLOG_H

#include <iostream>
#include <streambuf>
#include <string>

#include <QtGui>


/** \addtogroup libGUI
 *  @{
 */

class StreamLog : public std::basic_streambuf<char>{
  public:
    StreamLog(std::ostream &stream, QTextEdit* text_edit) : m_stream(stream) {
      log_window = text_edit;
      m_old_buf = stream.rdbuf();
      stream.rdbuf(this);
    }
    ~StreamLog() {
      // output anything that is left
       if (!m_string.empty())
         log_window->append(m_string.c_str());

       m_stream.rdbuf(m_old_buf);
     }

 protected:
   virtual int_type overflow(int_type v) {
     if (v == '\n'){
       log_window->append(m_string.c_str());
       m_string.erase(m_string.begin(), m_string.end());
     }else
       m_string += v;

    return v;
   }

  virtual std::streamsize xsputn(const char *p, std::streamsize n) {
    m_string.append(p, p + n);

    int pos = 0;
    while (pos != std::string::npos){
      pos = m_string.find('\n');
      if (pos != std::string::npos) {
        std::string tmp(m_string.begin(), m_string.begin() + pos);
        log_window->append(tmp.c_str());
        m_string.erase(m_string.begin(), m_string.begin() + pos + 1);
      }
    }

    return n;
  }

  private:
    std::ostream &m_stream;
    std::streambuf *m_old_buf;
    std::string m_string;
    QTextEdit* log_window;
};


/** @}   end of Doxygen module "libGUI" */
#endif // STREAMLOG_H
