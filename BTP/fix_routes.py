import xml.etree.ElementTree as ET
import sys
import os

def fix_inline_routes(file_path):
    print(f"Processing {file_path}...")
    try:
        tree = ET.parse(file_path)
    except Exception as e:
        print(f"Could not parse {file_path}: {e}")
        return
    
    root = tree.getroot()
    changed = 0
    
    # We need to iterate over a copy of children since we'll be inserting new elements
    for i, elem in enumerate(list(root)):
        if elem.tag in ['vehicle', 'flow']:
            # Find inline route
            route = elem.find('route')
            if route is not None and 'edges' in route.attrib:
                # We have an inline route!
                vid = elem.attrib.get('id', 'unknown')
                route_id = f"r_{vid}"
                
                # Create standalone route
                new_route = ET.Element('route', {'id': route_id, 'edges': route.attrib['edges']})
                
                # Insert the new route immediately before the vehicle
                root.insert(i + changed, new_route)
                
                # Remove inline route
                elem.remove(route)
                
                # Set route attribute on the vehicle/flow
                elem.set('route', route_id)
                changed += 1

    if changed > 0:
        print(f"Fixed {changed} inline routes in {file_path}")
        tree.write(file_path, encoding="UTF-8", xml_declaration=True)
    else:
        print(f"No inline routes found in {file_path}")

if __name__ == "__main__":
    for f in sys.argv[1:]:
        fix_inline_routes(f)
