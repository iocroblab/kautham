Documentation
=============

`Doxygen documentation <https://sir.upc.edu/projects/kautham/doxygen_documentation/html/index.html>`_

`Slides used to explain the Kautham Project <files/slides THE KAUTHAM PROJECT.pdf>`_

`Slides explaining OMPL integration <files/slides OMPL-KAUTHAM integration.pdf>`_

`Paper of Kautham <https://ioc.upc.edu/ca/personal/jan.rosell/publications/papers/the-kautham-project-a-teaching-and-research-tool-for-robot-motion-planning/@@download/file/PID3287499.pdf>`_

**Examples of input files**

+----------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Problem definition                     |  `OMPL_RRTConnect_Staubli_R6_mobileBase_two_columns.xml <files/OMPL_RRTConnect_Staubli_R6_mobileBase_two_columns.xml>`_ |
+----------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Robot file (dh)                        |  `UR5.dh <files/UR5.dh>`_                                                                                               |
+----------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Robot file (urdf)                      |  `allegro_hand_description_right.urdf <files/allegro_hand_description_right.urdf>`_                                     |
+----------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Robot control file                     |  `TX90_mobile.cntr <files/TX90_mobile.cntr>`_                                                                           |
+----------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Robot control file with coupled d.o.f. |  `TX90_6dof_RHand_5PMD.cntr <files/TX90_6dof_RHand_5PMD.cntr>`_                                                         |
+----------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Obstacle file (static)                 |  `shelf <files/shelf>`_                                                                                                 |
+----------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Obstacle file with dynamic data        |  `columns.urdf <files/columns.urdf>`_                                                                                   |
+----------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Obstacle file (a robot)                |  `TX90.dh <files/TX90.dh>`_                                                                                             |
+----------------------------------------+-------------------------------------------------------------------------------------------------------------------------+

**Citation** 

If you use Kautham in publication, please use: ::

    @inproceedings{kautham2014,
    author = {Jan Rosell and
                Alexander P\'erez and
                Akbari Aliakbar and
                Muhayyuddin and
                Leopold Palomo and 
                N\'estor Garc\'{\i}a},
    title = {The Kautham Project: A teaching and research tool for robot motion planning},
    booktitle={Proc. of the IEEE Int. Conf. on Emerging Technologies and Factory Automation, ETFA'14},
    year = {2014},
    url = {sir.upc.edu/kautham}
    }
