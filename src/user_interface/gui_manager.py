#!/usr/bin/env python3
"""
GUI Manager for LLM-Cobot Project
Simple interface with speech-to-text and text input
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import pyaudio
import wave
import tempfile
import os
from groq import Groq
from dotenv import load_dotenv
import time

load_dotenv()

class RobotGUI:
    """
    Simple GUI for the LLM-Cobot system with speech-to-text and text input
    """
    
    def __init__(self, llm_manager=None, api_bridge=None):
        self.llm_manager = llm_manager
        self.api_bridge = api_bridge
        
        # Groq client for speech-to-text
        self.groq_client = Groq(api_key=os.getenv('GROQ_API_KEY'))
        
        # Recording variables
        self.is_recording = False
        self.audio_frames = []
        self.audio_stream = None
        self.audio = pyaudio.PyAudio()
        
        # Audio settings
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000  # Whisper works well with 16kHz
        
        self.setup_gui()
        
    def setup_gui(self):
        """Create the main GUI window"""
        self.root = tk.Tk()
        self.root.title("LLM-Cobot Control Interface")
        self.root.geometry("800x700")
        self.root.configure(bg='#f0f0f0')
        
        # Create main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # Title
        title_label = ttk.Label(main_frame, text="🤖 LLM-Cobot Control Interface", 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # Speech Input Section
        speech_frame = ttk.LabelFrame(main_frame, text="🎤 Speech Input", padding="10")
        speech_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        speech_frame.columnconfigure(1, weight=1)
        
        # Recording button
        self.record_button = ttk.Button(speech_frame, text="🎤 Start Recording", 
                                       command=self.toggle_recording)
        self.record_button.grid(row=0, column=0, padx=(0, 10))
        
        # Recording status
        self.status_label = ttk.Label(speech_frame, text="Ready to record", 
                                     foreground="green")
        self.status_label.grid(row=0, column=1, sticky=tk.W)
        
        # Transcription display
        ttk.Label(speech_frame, text="Transcription:").grid(row=1, column=0, sticky=tk.W, pady=(10, 0))
        self.transcription_text = scrolledtext.ScrolledText(speech_frame, height=3, wrap=tk.WORD)
        self.transcription_text.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(5, 0))
        
        # Text Input Section
        text_frame = ttk.LabelFrame(main_frame, text="⌨️ Text Input", padding="10")
        text_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        text_frame.columnconfigure(0, weight=1)
        
        # Text input field
        self.text_input = ttk.Entry(text_frame, font=('Arial', 12))
        self.text_input.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=(0, 10))
        self.text_input.bind('<Return>', self.send_text_command)
        
        # Send button
        self.send_button = ttk.Button(text_frame, text="Send", command=self.send_text_command)
        self.send_button.grid(row=0, column=1)
        
        # Communication Feed
        comm_frame = ttk.LabelFrame(main_frame, text="💬 Communication Feed", padding="10")
        comm_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        comm_frame.columnconfigure(0, weight=1)
        comm_frame.rowconfigure(0, weight=1)
        
        # Communication display
        self.comm_display = scrolledtext.ScrolledText(comm_frame, height=15, wrap=tk.WORD, 
                                                     font=('Arial', 10))
        self.comm_display.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure text tags for different message types
        self.comm_display.tag_configure("user", foreground="blue", font=('Arial', 10, 'bold'))
        self.comm_display.tag_configure("robot", foreground="green", font=('Arial', 10))
        self.comm_display.tag_configure("action", foreground="purple", font=('Arial', 10, 'italic'))
        self.comm_display.tag_configure("error", foreground="red", font=('Arial', 10))
        self.comm_display.tag_configure("timestamp", foreground="gray", font=('Arial', 8))
        
        # Control Buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=4, column=0, columnspan=2, pady=(10, 0))
        
        ttk.Button(button_frame, text="Clear Feed", command=self.clear_feed).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(button_frame, text="Emergency Stop", command=self.emergency_stop, 
                  style="Emergency.TButton").pack(side=tk.LEFT)
        
        # Configure emergency button style
        style = ttk.Style()
        style.configure("Emergency.TButton", foreground="red")
        
        # Initial message
        self.add_to_feed("System", "🤖 LLM-Cobot system ready!", "robot")
        
    def toggle_recording(self):
        """Start or stop audio recording"""
        if not self.is_recording:
            self.start_recording()
        else:
            self.stop_recording()
    
    def start_recording(self):
        """Start recording audio"""
        try:
            self.is_recording = True
            self.audio_frames = []
            
            # Update UI
            self.record_button.config(text="🛑 Stop Recording")
            self.status_label.config(text="Recording...", foreground="red")
            
            # Start audio stream
            self.audio_stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )
            
            # Start recording in separate thread
            self.recording_thread = threading.Thread(target=self._record_audio)
            self.recording_thread.start()
            
        except Exception as e:
            messagebox.showerror("Recording Error", f"Failed to start recording: {str(e)}")
            self.is_recording = False
            self.record_button.config(text="🎤 Start Recording")
            self.status_label.config(text="Ready to record", foreground="green")
    
    def stop_recording(self):
        """Stop recording and process audio"""
        if not self.is_recording:
            return
            
        self.is_recording = False
        
        # Update UI
        self.record_button.config(text="🎤 Start Recording")
        self.status_label.config(text="Processing...", foreground="orange")
        
        # Process audio in separate thread
        threading.Thread(target=self._process_audio).start()
    
    def _record_audio(self):
        """Record audio frames (runs in separate thread)"""
        while self.is_recording:
            try:
                data = self.audio_stream.read(self.chunk)
                self.audio_frames.append(data)
            except Exception as e:
                print(f"Recording error: {e}")
                break
    
    def _process_audio(self):
        """Process recorded audio and transcribe (runs in separate thread)"""
        try:
            # Stop and close audio stream
            if self.audio_stream:
                self.audio_stream.stop_stream()
                self.audio_stream.close()
            
            if not self.audio_frames:
                self.status_label.config(text="No audio recorded", foreground="red")
                return
            
            # Save audio to temporary file
            temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
            
            with wave.open(temp_file.name, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(self.audio.get_sample_size(self.format))
                wf.setframerate(self.rate)
                wf.writeframes(b''.join(self.audio_frames))
            
            # Transcribe using Groq Whisper
            with open(temp_file.name, "rb") as file:
                transcription = self.groq_client.audio.transcriptions.create(
                    file=(temp_file.name, file.read()),
                    model="whisper-large-v3-turbo",
                    response_format="text"
                )
            
            # Clean up temp file
            os.unlink(temp_file.name)
            
            # Update UI with transcription
            self.root.after(0, self._update_transcription, transcription)
            
        except Exception as e:
            self.root.after(0, self._transcription_error, str(e))
    
    def _update_transcription(self, transcription):
        """Update transcription display and process command"""
        self.transcription_text.delete(1.0, tk.END)
        self.transcription_text.insert(1.0, transcription)
        
        self.status_label.config(text="Transcription complete", foreground="green")
        
        # Process the transcribed command
        if transcription.strip():
            self.process_command(transcription.strip())
    
    def _transcription_error(self, error):
        """Handle transcription errors"""
        self.status_label.config(text=f"Transcription failed: {error}", foreground="red")
        messagebox.showerror("Transcription Error", f"Failed to transcribe audio: {error}")
    
    def send_text_command(self, event=None):
        """Send text command"""
        command = self.text_input.get().strip()
        if command:
            self.text_input.delete(0, tk.END)
            self.process_command(command)
    
    def process_command(self, command):
        """Process user command through LLM and robot system"""
        self.add_to_feed("User", command, "user")
        
        # Terminal output like the old system
        print(f"\n🗣️ Processing: '{command}'")
        
        if not self.llm_manager or not self.api_bridge:
            self.add_to_feed("System", "❌ LLM Manager or API Bridge not connected", "error")
            return
        
        # Process in separate thread to avoid blocking UI
        threading.Thread(target=self._process_command_async, args=(command,)).start()

    def _process_command_async(self, command):
        """Process command with terminal output matching old system"""
        try:
            # Process with LLM
            llm_response = self.llm_manager.process_command(command)
            
            if not llm_response.success:
                print(f"❌ LLM Error: {llm_response.feedback}")
                self.root.after(0, self.add_to_feed, "System", f"❌ {llm_response.feedback}", "error")
                return
            
            # Terminal output - same as old system
            print(f"🧠 LLM Understanding: {llm_response.understanding}")
            
            # Show response FIRST (before executing actions) - like old system
            if llm_response.feedback:
                print(f"🤖 Response: {llm_response.feedback}")
                # Also add to GUI feed (clean version)
                self.root.after(0, self.add_to_feed, "Robot", llm_response.feedback, "robot")
            
            # Then check if there are actions to execute
            if llm_response.actions:
                print(f"🎯 Actions: {llm_response.actions}")
                print("🤖 Executing on robot...")
                
                # Execute via API bridge
                action_results = self.api_bridge.execute_action_list(llm_response.actions)
                
                # Show results in terminal - like old system
                for i, result in enumerate(action_results, 1):
                    if result["result"].value == "success":
                        print(f"✅ Action {i}: {result['message']}")
                    else:
                        print(f"❌ Action {i}: {result['message']}")
                        # Only show errors in GUI feed
                        self.root.after(0, self.add_to_feed, "System", f"Action failed: {result['message']}", "error")
                        break
            else:
                # No actions - just a question/answer
                print("💭 No robot actions needed")
                
        except Exception as e:
            error_msg = f"❌ Error processing command: {str(e)}"
            print(error_msg)
            self.root.after(0, self.add_to_feed, "System", error_msg, "error")
    
    def add_to_feed(self, sender, message, tag="robot"):
        """Add message to communication feed"""
        timestamp = time.strftime("%H:%M:%S")
        
        self.comm_display.insert(tk.END, f"[{timestamp}] ", "timestamp")
        self.comm_display.insert(tk.END, f"{sender}: ", tag)
        self.comm_display.insert(tk.END, f"{message}\n")
        
        # Auto-scroll to bottom
        self.comm_display.see(tk.END)
    
    def clear_feed(self):
        """Clear the communication feed"""
        self.comm_display.delete(1.0, tk.END)
        self.add_to_feed("System", "Communication feed cleared", "robot")
    
    def emergency_stop(self):
        """Emergency stop function"""
        self.add_to_feed("System", "🚨 EMERGENCY STOP ACTIVATED", "error")
        # TODO: Implement actual emergency stop for robot
        messagebox.showwarning("Emergency Stop", "Emergency stop activated!\n(Robot stop functionality to be implemented)")
    
    def run(self):
        """Start the GUI main loop"""
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.mainloop()
        except KeyboardInterrupt:
            self.on_closing()
    
    def on_closing(self):
        """Handle window closing"""
        if self.is_recording:
            self.stop_recording()
        
        # Cleanup audio
        if hasattr(self, 'audio'):
            self.audio.terminate()
        
        self.root.destroy()

# Example usage and testing
def main():
    """Test the GUI (without robot connection)"""
    # Create mock managers for testing
    class MockLLMManager:
        def process_command(self, command):
            from llm_framework.llm_manager import LLMResponse
            return LLMResponse(
                understanding=f"Mock understanding of: {command}",
                actions=["MOVE(0.3, 0.2, 0.1)"] if "move" in command.lower() else [],
                feedback="This is a mock response for testing",
                raw_response="Mock raw response",
                success=True
            )
    
    class MockAPIBridge:
        def execute_action_list(self, actions):
            from llm_framework.action_library import ActionResult
            return [{"result": ActionResult.SUCCESS, "message": f"Mock executed: {action}"} for action in actions]
    
    # Create and run GUI
    gui = RobotGUI(MockLLMManager(), MockAPIBridge())
    print("🖥️ Starting GUI...")
    gui.run()

if __name__ == '__main__':
    main()