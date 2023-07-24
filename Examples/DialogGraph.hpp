/*****************************************************************************\
*                                                                           *
* File(s): exampleMinimumGraphicsSetup.hpp                                            *
*                                                                           *
* Content: Example scene that shows minimum setup with an OpenGL capable   *
*          window, lighting setup, and a single moving object.              *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_DIALOGGRAPH_HPP__
#define __CFORGE_DIALOGGRAPH_HPP__



using namespace std;

namespace CForge {

	class Dialoggraph {
	public:
		Dialoggraph(void) {
			
		}//Constructor

		~Dialoggraph(void) {
			
		}//Destructor

		bool playerSpeaking;
		string text;
		vector <Dialoggraph> answers;


	protected:
		

	};//Dialoggraph

}//name space

#endif