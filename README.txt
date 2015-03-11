####################################################################
#                                                                  #
#    ObjRecRANSAC - RANSAC variant for 3D object recognition in    #
#                   occluded scenes                                #
#                                                                  #
#    Chavdar Papazov (papazov@in.tum.de)                           #
#    Darius Burschka (burschka@in.tum.de)                          #
#                                                                  #
####################################################################

1. Introduction.

Thank you for taking interest in our work and downloading this
software. This software implements a modified version of the
algorithm described in the papers

 * Chavdar Papazov and Darius Burschka: "An Efficient RANSAC for 3D
   Object Recognition in Noisy and Occluded Scenes". In Proceedings
   of the 10th Asian Conference on Computer Vision (ACCV'10), 2010

 * Chavdar Papazov, Sami Haddadin, Sven Parusel, Kai Krieger, and
   Darius Burschka: "Rigid 3D Geometry Matching for Grasping of
   Known Objects in Cluttered Scenes". International Journal of
   Robotics Research, 31, April 2012.

If you use this software please cite the aforementioned papers in
any resulting publication.

Please send questions, comments and/or bug reports to
Chavdar Papazov (papazov@in.tum.de)

The software was tested on 64bit Linux (Fedora 12 and Ubuntu 11.10).

##################################################################

2. License & disclaimer.

    Copyright 2011-212 Chavdar Papazov (papazov@in.tum.de)
                       Darius Burschka (burschka@in.tum.de)

    This software may be used for research purposes only.

	Do not redistribute.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  This software contains source code provided by NVIDIA Corporation.

##################################################################

3. How to build and run.

   1. Install cmake (www.cmake.org), VTK (www.vtk.org) and OpenCV
   2. Go to the folder build (using the command line)
   3. Type ccmake ..
      3.1. Press c (for configure)
      3.2. Set CMAKE_BUILD_TYPE to Release
      3.3. Press c again
      3.4. Press g (for generate)
   4. Type make
   5. Type ./ObjRecRANSACRun to run the program
   6. After the computations are performed a window should pop up
      showing the result of the recognition. It should be similar
      to the image typical_result.png in the main folder. You can
      navigate in the scene using the mouse.

##################################################################

4. Some additional comments.

The image typical_result.png shows a result of the recognition using
the data sets in the data folder. The gray points show the detected
plane and are not used for the recognition. The blue points are the
non-plane points used for the recognition. The orange meshes are
representing the recognized model instances in the scene.

Check the main.cpp file to see how to use the algorithm. Note that
the recognition is based on a stochastic method, i.e., it produces
each time a different result.

