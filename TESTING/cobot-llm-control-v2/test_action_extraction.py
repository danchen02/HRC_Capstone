#!/usr/bin/env python3
"""
Test to verify action extraction only looks at Actions section
"""

import sys
sys.path.insert(0, '/mnt/user-data/outputs')
from llm_evaluator import SimpleLLMEvaluator

# Create test responses
test_responses = [
    {
        "name": "Actions section empty but mentions PICK in Understanding",
        "response": """Understanding: The user wants me to PICK up the red cube and SCAN the area
Actions: 
Feedback: I cannot complete this task because the workspace needs to be scanned first"""
    },
    {
        "name": "Actions section has MOVE, other sections mention PICK",
        "response": """Understanding: I should PICK up the object first
Actions: MOVE(0.3, 0.2, 0.1)
Feedback: After moving, I can SCAN the workspace"""
    },
    {
        "name": "Actions section has multiple actions",
        "response": """Understanding: The user wants to stack objects
Actions: PICK(cube_002), PLACE(0.3, 0.3, 0.1)
Feedback: I will pick up the cube and place it"""
    },
    {
        "name": "Actions section truly empty (ambiguous command)",
        "response": """Understanding: The command is unclear
Actions:
Feedback: Could you please specify which object you want me to move?"""
    }
]

print("=" * 70)
print("🧪 TESTING ACTION EXTRACTION FIX")
print("=" * 70)
print("\nVerifying actions are ONLY extracted from 'Actions:' section\n")

evaluator = SimpleLLMEvaluator()

for i, test in enumerate(test_responses, 1):
    print(f"\nTest {i}: {test['name']}")
    print("-" * 70)
    print(test['response'])
    print("-" * 70)
    
    actions = evaluator.extract_actions(test['response'])
    
    print(f"Extracted actions: {actions}")
    
    # Check results
    if i == 1:
        expected = []
        status = "✅ PASS" if actions == expected else "❌ FAIL"
        print(f"{status} - Should find NO actions (Actions section is empty)")
    elif i == 2:
        expected = ['MOVE']
        status = "✅ PASS" if actions == expected else "❌ FAIL"
        print(f"{status} - Should find only MOVE (not PICK/SCAN from other sections)")
    elif i == 3:
        expected = set(['PICK', 'PLACE'])
        status = "✅ PASS" if set(actions) == expected else "❌ FAIL"
        print(f"{status} - Should find PICK and PLACE")
    elif i == 4:
        expected = []
        status = "✅ PASS" if actions == expected else "❌ FAIL"
        print(f"{status} - Should find NO actions (asking for clarification)")
    
    print()

print("=" * 70)
print("✅ Test complete! The fix ensures actions are only extracted from")
print("   the Actions: section, not from Understanding: or Feedback:")
print("=" * 70)