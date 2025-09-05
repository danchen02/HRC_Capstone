#!/usr/bin/env python3
"""
LLM Framework Module for LLM-Cobot Project
Simplified version focused on robot control communication
"""

import yaml
import os
from groq import Groq
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from dotenv import load_dotenv

load_dotenv()

@dataclass
class LLMResponse:
    """Response from LLM processing"""
    understanding: str
    actions: List[str]
    feedback: str
    raw_response: str
    success: bool

@dataclass
class ConversationTurn:
    """Single conversation turn"""
    user_input: str
    understanding: str
    actions: List[str]
    feedback: str

class LLMManager:
    """
    Simple LLM manager for robot control with conversation memory.
    Reads object database and communicates with Groq API.
    """
    
    def __init__(self, objects_file: str = "config/objects.yaml", motion_planner=None):
        # API setup
        self.api_key = os.getenv('GROQ_API_KEY')
        if not self.api_key:
            raise ValueError("GROQ_API_KEY not found in environment variables")
        
        self.client = Groq(api_key=self.api_key)
        
        # Use single model
        self.model_id = "llama-3.3-70b-versatile"
        
        # Object database
        self.objects_file = objects_file
        self.objects_data = {}
        self.load_objects()
        
        # Robot control settings
        self.max_tokens = 512
        self.temperature = 0.3

        self.motion_planner = motion_planner
        
        # Simple memory: last 5 conversations
        self.conversation_history: List[ConversationTurn] = []
        self.max_history = 5
        
        print(f"✅ LLM Manager initialized with llama-3.3-70b-versatile")
        print(f"📁 Loaded {len(self.objects_data.get('objects', {}))} objects from database")
    
    def load_objects(self):
        """Load object database from YAML file"""
        try:
            if os.path.exists(self.objects_file):
                with open(self.objects_file, 'r') as f:
                    self.objects_data = yaml.safe_load(f)
                print(f"📂 Loaded object database: {self.objects_file}")
            else:
                print(f"⚠️  Object database not found: {self.objects_file}")
                # Create empty structure
                self.objects_data = {"objects": {}, "metadata": {}}
        except Exception as e:
            print(f"❌ Error loading object database: {e}")
            self.objects_data = {"objects": {}, "metadata": {}}
    
    def get_objects_context(self) -> str:
        """Create context string about available objects"""
        if not self.objects_data.get('objects'):
            return "No objects currently detected in workspace."
        
        context = "Current objects in workspace:\n"
        for obj_id, obj_info in self.objects_data['objects'].items():
            name = obj_info.get('name', obj_id)
            desc = obj_info.get('description', 'no description')
            pos = obj_info.get('position', {})
            graspable = obj_info.get('properties', {}).get('graspable', True)
            
            context += f"- {name}: {desc}"
            if pos:
                context += f" at ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f}, {pos.get('z', 0):.2f})"
            if not graspable:
                context += " (not graspable)"
            context += "\n"
        
        return context
    
    def get_conversation_context(self) -> str:
        """Create conversation history context"""
        if not self.conversation_history:
            return "No previous conversation."
        
        context = "Recent conversation:\n"
        for turn in self.conversation_history:
            context += f"User: {turn.user_input}\n"
            context += f"Understanding: {turn.understanding}\n"
            context += f"Actions: {turn.actions}\n"
            context += f"Response: {turn.feedback}\n"
            context += "---\n"
        
        return context
    
    def create_system_prompt(self) -> str:
        """Create system prompt with current object context and conversation memory"""
        objects_context = self.get_objects_context()
        conversation_context = self.get_conversation_context()

        current_pos = "unknown"
        if self.motion_planner:
            current_pos = self.motion_planner.get_current_position()
        
        prompt = f"""You are a UR3 robot arm in a collaborative workspace.

AVAILABLE ACTIONS:
- MOVE(x, y, z): Move end-effector to coordinates
- PICK(object_name): Pick up specified object, dont move to object
- PLACE(x, y, z): Place held object at coordinates, if location not fully specified, place on floor
- HOME(): Move robot to home position (safe starting position)
- SCAN(): Scan workspace to update object detection
- WAIT(seconds): Wait for specified time
- QUERY(question): Ask human for clarification

Current end-effector position: {current_pos}

CURRENT WORKSPACE:
{objects_context}

CONVERSATION MEMORY:
{conversation_context}

RESPONSE FORMAT:
Understanding: [what you think the user wants]
Actions: [specific actions to take, e.g., MOVE(0.3, 0.2, 0.1) OR leave empty if just answering a question]
Feedback: [any questions or status updates for the user OR direct answers to general questions]

IMPORTANT:
- For general knowledge questions (non-robot related), provide the answer directly in the Feedback section and leave Actions empty
- Use exact object names from the workspace list
- Coordinates should be within robot reach (-0.8 to 0.8 for x,y, 0 to 1.0 for z)
- Consider previous conversation when interpreting current request ingnoring coordinates (only use item locations from "CURRENT WORKSPACE")
- If unclear, ask for clarification with QUERY()
- If you use QUERY(), it must be the ONLY action. Never combine QUERY with other actions
- If object not found, suggest SCAN() first
- if asked to PICK() or PLACE() object, don't MOVE() to location"""
        
        return prompt
    
    def process_command(self, user_command: str) -> LLMResponse:
        """
        Process user command and return structured response
        
        Args:
            user_command: Natural language command from user
            
        Returns:
            LLMResponse with parsed understanding, actions, and feedback
        """
        try:
            # Reload objects to get latest data
            self.load_objects()
            
            # Create system prompt with current context and memory
            system_prompt = self.create_system_prompt()
            
            # Call LLM API
            response = self.client.chat.completions.create(
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_command}
                ],
                model=self.model_id,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
            )
            
            raw_response = response.choices[0].message.content
            
            # Parse response
            parsed = self.parse_response(raw_response)
            
            # Store conversation turn in memory
            self.add_to_memory(
                user_command, 
                parsed['understanding'], 
                parsed['actions'],
                parsed['feedback']
            )
            
            return LLMResponse(
                understanding=parsed['understanding'],
                actions=parsed['actions'],
                feedback=parsed['feedback'],
                raw_response=raw_response,
                success=True
            )
            
        except Exception as e:
            print(f"❌ LLM processing error: {e}")
            return LLMResponse(
                understanding="Error processing command",
                actions=[],
                feedback=f"Sorry, I encountered an error: {str(e)}",
                raw_response="",
                success=False
            )
    
    def add_to_memory(self, user_input: str, understanding: str, actions: List[str], feedback: str):
        """Add conversation turn to memory"""
        turn = ConversationTurn(
            user_input=user_input,
            understanding=understanding,
            actions=actions,
            feedback=feedback
        )
        
        self.conversation_history.append(turn)
        
        # Keep only last 5 conversations
        if len(self.conversation_history) > self.max_history:
            self.conversation_history = self.conversation_history[-self.max_history:]
    
    def parse_response(self, response_text: str) -> Dict[str, Any]:
        """Parse LLM response into structured format"""
        understanding = ""
        actions = []
        feedback = ""
        
        lines = response_text.split('\n')
        current_section = None
        
        for line in lines:
            line = line.strip()
            
            if line.startswith('Understanding:'):
                current_section = 'understanding'
                understanding = line.replace('Understanding:', '').strip()
            elif line.startswith('Actions:'):
                current_section = 'actions'
                actions_text = line.replace('Actions:', '').strip()
                if actions_text:
                    actions.extend(self.extract_actions(actions_text))
            elif line.startswith('Feedback:'):
                current_section = 'feedback'
                feedback = line.replace('Feedback:', '').strip()
            elif current_section and line:
                # Continue previous section
                if current_section == 'understanding':
                    understanding += " " + line
                elif current_section == 'actions':
                    actions.extend(self.extract_actions(line))
                elif current_section == 'feedback':
                    feedback += " " + line
        
        # Fallback: extract actions from anywhere in response if not found
        if not actions:
            actions = self.extract_actions(response_text)
        
        return {
            'understanding': understanding.strip(),
            'actions': actions,
            'feedback': feedback.strip()
        }
    
    def extract_actions(self, text: str) -> List[str]:
        """Extract action commands from text in the order they appear"""
        import re
        
        action_keywords = ["MOVE", "PICK", "PLACE", "HOME", "SCAN", "WAIT", "QUERY"]
        
        # Find all actions with their positions in the text
        action_matches = []
        
        for keyword in action_keywords:
            # Pattern: ACTION(params) or just ACTION
            pattern = rf'{keyword}\s*(?:\([^)]*\))?'
            
            for match in re.finditer(pattern, text.upper()):
                action_matches.append({
                    'action': match.group(),
                    'position': match.start(),
                    'keyword': keyword
                })
        
        # Sort by position in text to preserve order
        action_matches.sort(key=lambda x: x['position'])
        
        # Extract just the action strings
        return [match['action'] for match in action_matches]
    
    def generate_feedback(self, action_result: str, error_type: Optional[str] = None) -> str:
        """
        Generate natural language feedback based on action results
        
        Args:
            action_result: Result message from action execution
            error_type: Type of error if any
            
        Returns:
            Natural language feedback for user
        """
        if error_type:
            feedback_prompt = f"""The robot action failed with error: {action_result}
            
Generate a helpful, natural response to tell the user what went wrong and suggest next steps.
Keep it conversational and under 50 words."""
        else:
            feedback_prompt = f"""The robot successfully completed: {action_result}
            
Generate a brief, natural confirmation message for the user.
Keep it conversational and under 30 words."""
        
        try:
            response = self.client.chat.completions.create(
                messages=[
                    {"role": "system", "content": "You are a helpful robot assistant. Generate natural, conversational responses."},
                    {"role": "user", "content": feedback_prompt}
                ],
                model=self.model_id,
                temperature=0.5,
                max_tokens=100,
            )
            
            return response.choices[0].message.content.strip()
            
        except Exception as e:
            # Fallback to simple message
            if error_type:
                return f"Something went wrong: {action_result}"
            else:
                return f"Done! {action_result}"
    
# Example usage and testing
def main():
    """Test the LLM framework"""
    try:
        # Initialize LLM manager
        llm = LLMManager()
        
        # Test commands
        test_commands = [
            "Pick up the hammer",
            "Move to position 0.3, 0.2, 0.1", 
            "Scan the workspace",
            "What tools are available?",
            "Place the object on the table"
            "Now move back to where the hammer was",
            "What did I ask you to do first?",
            "Pick up that cube we talked about earlier"
        ]
        
        print(f"\n{'='*50}")
        print("🤖 TESTING LLM FRAMEWORK")
        print(f"{'='*50}")
        
        for i, command in enumerate(test_commands, 1):
            print(f"\n[{i}] User: {command}")
            
            response = llm.process_command(command)
            
            if response.success:
                print(f"Understanding: {response.understanding}")
                print(f"Actions: {response.actions}")
                print(f"Feedback: {response.feedback}")
            else:
                print(f"❌ Error: {response.feedback}")
            
            print("-" * 40)
        
    except Exception as e:
        print(f"❌ Error: {e}")


if __name__ == '__main__':
    main()