#!/usr/bin/env python3
"""
API Bridge Module for LLM-Cobot Project
Parses LLM action strings and routes to Action Library
"""

import re
from typing import List, Dict, Optional, Any
from enum import Enum

# Import action library
from llm_framework.action_library import ActionLibrary, ActionResult

class ParseResult(Enum):
    """Result of string parsing"""
    SUCCESS = "success"
    INVALID_FORMAT = "invalid_format"
    UNKNOWN_ACTION = "unknown_action"


class APIBridge:
    """
    Simple bridge that parses LLM action strings and routes to ActionLibrary.
    Only handles string parsing - no robot logic.
    """
    
    def __init__(self, action_library: ActionLibrary):
        self.action_library = action_library
        print("🔗 API Bridge initialized")
    
    def execute_action_list(self, action_strings: List[str]) -> List[Dict[str, Any]]:
        """
        Execute a list of action strings from LLM.
        
        Args:
            action_strings: List like ["MOVE(0.3, 0.2, 0.1)", "PICK(hammer)"]
            
        Returns:
            List of execution results
        """
        results = []
        
        for action_string in action_strings:
            print(f"🔄 Processing: {action_string}")
            
            # Parse the action string
            parsed = self.parse_action_string(action_string)
            
            if parsed["parse_result"] != ParseResult.SUCCESS:
                # Parsing failed
                results.append({
                    "result": ActionResult.FAILED,
                    "message": parsed["message"],
                    "action": action_string
                })
                print(f"❌ Parse failed: {parsed['message']}")
                break
            
            # Execute via action library
            execution_result = self.execute_parsed_action(parsed)
            execution_result["action"] = action_string
            results.append(execution_result)
            
            # Stop on first failure
            if execution_result["result"] != ActionResult.SUCCESS:
                print(f"❌ Execution failed: {execution_result['message']}")
                break
            else:
                print(f"✅ {execution_result['message']}")
        
        return results
    
    def parse_action_string(self, action_string: str) -> Dict[str, Any]:
        """
        Parse a single action string into action type and parameters.
        
        Args:
            action_string: String like "MOVE(0.3, 0.2, 0.1)"
            
        Returns:
            Dict with parsing results
        """
        action_string = action_string.strip().upper()
        
        if action_string.startswith("MOVE"):
            return self._parse_move(action_string)
        elif action_string.startswith("PICK"):
            return self._parse_pick(action_string)
        elif action_string.startswith("PLACE"):
            return self._parse_place(action_string)
        elif action_string.startswith("SCAN"):
            return self._parse_scan(action_string)
        elif action_string.startswith("WAIT"):
            return self._parse_wait(action_string)
        elif action_string.startswith("HOME"):
            return self._parse_home(action_string)
        else:
            return {
                "parse_result": ParseResult.UNKNOWN_ACTION,
                "message": f"Unknown action: {action_string}",
                "action_type": "unknown"
            }
    
    def execute_parsed_action(self, parsed: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute a parsed action via ActionLibrary.
        
        Args:
            parsed: Parsed action from parse_action_string()
            
        Returns:
            Execution result
        """
        action_type = parsed["action_type"]
        params = parsed.get("parameters", {})
        
        try:
            if action_type == "move":
                return self.action_library.execute_move(**params)
            elif action_type == "pick":
                return self.action_library.execute_pick(**params)
            elif action_type == "place":
                return self.action_library.execute_place(**params)
            elif action_type == "scan":
                return self.action_library.execute_scan(**params)
            elif action_type == "wait":
                return self.action_library.execute_wait(**params)
            elif action_type == "home":
                return self.action_library.execute_home(**params)
            else:
                return {
                    "result": ActionResult.FAILED,
                    "message": f"No handler for action type: {action_type}"
                }
                
        except Exception as e:
            return {
                "result": ActionResult.FAILED,
                "message": f"Execution error: {str(e)}"
            }
    
    def _parse_move(self, action_string: str) -> Dict[str, Any]:
        """Parse MOVE(x, y, z)"""
        # Extract coordinates from "MOVE(0.3, 0.2, 0.1)"
        numbers = re.findall(r'-?\d+\.?\d*', action_string)
        
        if len(numbers) < 3:
            return {
                "parse_result": ParseResult.INVALID_FORMAT,
                "message": f"MOVE needs 3 coordinates, got: {action_string}",
                "action_type": "move"
            }
        
        try:
            x, y, z = float(numbers[0]), float(numbers[1]), float(numbers[2])
            return {
                "parse_result": ParseResult.SUCCESS,
                "action_type": "move",
                "parameters": {"x": x, "y": y, "z": z}
            }
        except ValueError as e:
            return {
                "parse_result": ParseResult.INVALID_FORMAT,
                "message": f"Invalid coordinates in: {action_string}",
                "action_type": "move"
            }
    
    def _parse_pick(self, action_string: str) -> Dict[str, Any]:
        """Parse PICK(object_name)"""
        # Extract object name from "PICK(hammer)" or "PICK(HAMMER)"
        match = re.search(r'PICK\s*\(\s*([^)]+)\s*\)', action_string, re.IGNORECASE)
        
        if not match:
            return {
                "parse_result": ParseResult.INVALID_FORMAT,
                "message": f"Invalid PICK format: {action_string}",
                "action_type": "pick"
            }
        
        object_name = match.group(1).strip().strip('"\'').lower()
        
        return {
            "parse_result": ParseResult.SUCCESS,
            "action_type": "pick",
            "parameters": {"object_name": object_name}
        }
    
    def _parse_place(self, action_string: str) -> Dict[str, Any]:
        """Parse PLACE(x, y, z)"""
        # Extract coordinates from "PLACE(0.4, 0.3, 0.1)"
        numbers = re.findall(r'-?\d+\.?\d*', action_string)
        
        if len(numbers) < 3:
            return {
                "parse_result": ParseResult.INVALID_FORMAT,
                "message": f"PLACE needs 3 coordinates, got: {action_string}",
                "action_type": "place"
            }
        
        try:
            x, y, z = float(numbers[0]), float(numbers[1]), float(numbers[2])
            return {
                "parse_result": ParseResult.SUCCESS,
                "action_type": "place",
                "parameters": {"x": x, "y": y, "z": z}
            }
        except ValueError as e:
            return {
                "parse_result": ParseResult.INVALID_FORMAT,
                "message": f"Invalid coordinates in: {action_string}",
                "action_type": "place"
            }
    
    def _parse_scan(self, action_string: str) -> Dict[str, Any]:
        """Parse SCAN()"""
        return {
            "parse_result": ParseResult.SUCCESS,
            "action_type": "scan",
            "parameters": {}
        }
    
    def _parse_wait(self, action_string: str) -> Dict[str, Any]:
        """Parse WAIT(seconds)"""
        # Extract seconds from "WAIT(2.0)"
        numbers = re.findall(r'\d+\.?\d*', action_string)
        seconds = float(numbers[0]) if numbers else 1.0
        
        return {
            "parse_result": ParseResult.SUCCESS,
            "action_type": "wait",
            "parameters": {"seconds": seconds}
        }
    
    def _parse_home(self, action_string: str) -> Dict[str, Any]:
        """Parse HOME()"""
        return {
            "parse_result": ParseResult.SUCCESS,
            "action_type": "home",
            "parameters": {}
        }


# Test the API Bridge
def main():
    """Test the API Bridge parsing (without robot)"""
    
    # Mock action library for testing parsing
    class MockActionLibrary:
        def execute_move(self, x, y, z):
            return {"result": ActionResult.SUCCESS, "message": f"Mock move to ({x}, {y}, {z})"}
        def execute_pick(self, object_name):
            return {"result": ActionResult.SUCCESS, "message": f"Mock pick {object_name}"}
        def execute_scan(self):
            return {"result": ActionResult.SUCCESS, "message": "Mock scan"}
    
    # Test parsing
    bridge = APIBridge(MockActionLibrary())
    
    test_actions = [
        "MOVE(0.3, 0.2, 0.25)",
        "PICK(hammer)",
        "PLACE(0.4, 0.3, 0.15)",
        "SCAN()",
        "WAIT(2.0)",
        "HOME()",
        "INVALID_ACTION(test)"
    ]
    
    print("🔗 Testing API Bridge Parsing")
    print("=" * 50)
    
    for action in test_actions:
        print(f"\nTesting: {action}")
        parsed = bridge.parse_action_string(action)
        
        if parsed["parse_result"] == ParseResult.SUCCESS:
            print(f"✅ Parsed as {parsed['action_type']}: {parsed.get('parameters', {})}")
        else:
            print(f"❌ Parse failed: {parsed['message']}")


if __name__ == '__main__':
    main()