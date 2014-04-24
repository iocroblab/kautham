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


 

#include "application.h" 
#include <QApplication>
#include <QWidget>
#include <Inventor/Qt/SoQt.h>


int main(int argc, char* argv[]){       

    try{
        QWidget *app = SoQt::init(argv[0]);//argc, argv,argv[0]);
        app->setVisible(false);
        Application kauthApp;
        SoQt::mainLoop();
        return 0;
    }
    catch(...){
            std::cout <<"Unexpected error in Kautham initialization"<<endl;
        }
}

