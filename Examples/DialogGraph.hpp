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

#include "json/json.h"
#include <fstream>
#include <iostream>

using namespace std;

namespace CForge {

	class Dialoggraph {
	public:
		Dialoggraph(void) {
			
		}//Constructor

		~Dialoggraph(void) {
			
		}//Destructor

		void init(string filepath) {
			// read dialog from json file
			Json::Value root;
			std::ifstream ifs;
			ifs.open(filepath, std::ifstream::in);
			if (!ifs.is_open()) cout << "File could not be opened" << endl;
			Json::Reader reader;
			reader.parse(ifs, root);
			ifs.close();

			// initialize dialog graph
			text = root["text"].asString();
			playerSpeaking = root["playerSpeaking"].asBool();
			for (Json::Value i : root["answers"]) {
				Dialoggraph dialogInit;
				dialogInit.initAnswer(i);
				answers.push_back(dialogInit);				
			}		
		}

		void initAnswer(Json::Value answer) {
			text = answer["text"].asString();
			playerSpeaking = answer["playerSpeaking"].asBool();
			for (Json::Value j : answer["answers"]) {
				Dialoggraph dialogAnswer;
				dialogAnswer.initAnswer(j);
				answers.push_back(dialogAnswer);				
			}
		}

		bool playerSpeaking;
		string text;
		vector <Dialoggraph> answers;


	protected:
		

	};//Dialoggraph

}//name space

#endif