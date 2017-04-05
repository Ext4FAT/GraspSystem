#pragma once
#include <pxcsession.h>
#include <pxcspeechrecognition.h>
#include "Voice.h"
#include <string>
#include <memory>

using std::string;

namespace _IDLER_{
	typedef std::shared_ptr<pxcCHAR> pxcString;
	typedef PXCSpeechSynthesis::LanguageType LANGUAGE;

	class Speech{
	public:
		Speech() :session_(PXCSession::CreateInstance()), tts_(createSpeechSynthesis_(session_)), v_(genVoice_(tts_, PXCSpeechSynthesis::LANGUAGE_CN_CHINESE))
		{
		}
		~Speech(){
			session_->Release();
			tts_->Release();
		}
		int speak(string say){
			return speech_(tts_, v_, say);
		}
	private:
		pxcString string2PXCString_(string const& source)
		{
			int len = source.size() + 1;
			pxcString dest(new wchar_t[len]);
			MultiByteToWideChar(CP_ACP, 0, source.c_str(), -1, dest.get(), len);
			return dest;
		}
		PXCSpeechSynthesis* createSpeechSynthesis_(PXCSession *session)
		{
			PXCSpeechSynthesis *tts = 0;
			session->CreateImpl<PXCSpeechSynthesis>(&tts);
			return tts;
		}
		Voice genVoice_(PXCSpeechSynthesis* tts, LANGUAGE l = PXCSpeechSynthesis::LANGUAGE_CN_CHINESE)
		{
			PXCSpeechSynthesis::ProfileInfo pinfo;
			tts->QueryProfile(0, &pinfo);
			pinfo.language = l;
			pinfo.rate = 90;
			pinfo.volume = 90;
			tts->SetProfile(&pinfo);
			return Voice(&pinfo);
		}
		int speech_(PXCSpeechSynthesis* tts, Voice &v, string say = "Hello£¬Word! ÄãºÃ£¬ÊÀ½ç£¡")
		{
			tts->BuildSentence(1, string2PXCString_(say).get());
			int nbuffers = tts->QueryBufferNum(1);
			for (int i = 0; i < nbuffers; i++) {
				PXCAudio *audio = tts->QueryBuffer(1, i);
				v.RenderAudio(audio);
				//v.SaveFile(L"out.wav");
			}
			tts->ReleaseSentence(1);
			return 0;
		}
	private:
		PXCSession* session_ = 0;
		PXCSpeechSynthesis* tts_;
		Voice v_;
	};
}