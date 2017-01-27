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


#include <QtGui>


namespace Kautham {
    /** \addtogroup Application
    *  @{
    */

    /*!
    * \brief The DefaultPathDialog class allows the user to manage the default path list
    */
    class DefaultPathDialog : public QDialog {
        Q_OBJECT
    public:
        /*!
         * \brief DefaultPathDialog Constructs the dialog
         * \param pathList Path list to fill the dialog
         * \param parent Parent of the dialog
         * \param f Window flags
         */
        DefaultPathDialog(QStringList pathList, QWidget *parent = 0, Qt::WindowFlags f = 0);

        /*!
         * \brief getList Executes the dialog and lets the user to update the path list
         * \return path list defined by the user, NULL if the dialog was rejected by the user
         */
        QStringList *getList();

    private slots:
        /*!
         * \brief addDirectory Opens a file dialog and if a valid folder is selected it will be added to the list
         */
        void addDirectory();

        /*!
         * \brief removeDirectory Removes the current selected directory from the list
         */
        void removeDirectory();

        /*!
         * \brief upDirectory Moves up the current selected directory in the list
         */
        void upDirectory();

        /*!
         * \brief downDirectory Moves down the current selected directory in the list
         */
        void downDirectory();

    private:
        /*!
         * \brief pathListWidget list of paths
         */
        QListWidget *pathListWidget;
    };
    /** @}   end of Doxygen module "Application" */
}
