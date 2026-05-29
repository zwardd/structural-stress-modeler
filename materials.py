import math
from constants import MATERIAL_SPECS

# Profile types available for beams - all profile math centralized here
PROFILE_TYPES = ["Square Tube", "H-Beam", "Solid Bar"]


class MaterialManager:
    """
    Centralized material and geometric property calculations.
    All material science, cross-sectional properties, and profile handling 
    lives here to keep Beam class focused on structural network logic.
    """
    
    @staticmethod
    def get_material_specs(material_name):
        """Get full material spec dict (yield, ultimate, density, label, modulus)."""
        return MATERIAL_SPECS.get(material_name, MATERIAL_SPECS["Steel"])
    
    @staticmethod
    def get_yield_stress(material_name):
        """Get yield stress in Pa for a material."""
        specs = MATERIAL_SPECS.get(material_name, MATERIAL_SPECS["Steel"])
        return specs["yield"]
    
    @staticmethod
    def get_ultimate_stress(material_name):
        """Get ultimate tensile stress in Pa for a material."""
        specs = MATERIAL_SPECS.get(material_name, MATERIAL_SPECS["Steel"])
        return specs.get("ultimate", specs["yield"] * 1.6)
    
    @staticmethod
    def get_modulus(material_name):
        """Get Young's modulus in Pa for a material."""
        specs = MATERIAL_SPECS.get(material_name, MATERIAL_SPECS["Steel"])
        return specs.get("modulus", 200e9)  # Default to Steel modulus
    
    @staticmethod
    def get_density(material_name):
        """Get density in kg/m³ for a material."""
        specs = MATERIAL_SPECS.get(material_name, MATERIAL_SPECS["Steel"])
        return specs.get("density", 7850)  # Default to Steel density
    
    @staticmethod
    def calculate_area_inertia(profile, dim_w, dim_t):
        """
        Calculate cross-sectional area (m²) and moment of inertia (m⁴) 
        based on profile type and dimensions.
        
        Args:
            profile: One of "Square Tube", "H-Beam", "Solid Bar"
            dim_w: Width/diameter in meters
            dim_t: Thickness in meters (ignored for Solid Bar)
            
        Returns:
            tuple: (area in m², moment_of_inertia in m⁴)
        """
        if profile == "Solid Round":
            radius = dim_w / 2.0
            area = math.pi * (radius ** 2)
            inertia = (math.pi * (dim_w ** 4)) / 64.0
        elif profile == "Square Tube":
            outer_w = dim_w
            inner_w = max(0.0, dim_w - 2.0 * dim_t)
            area = (outer_w ** 2) - (inner_w ** 2)
            inertia = ((outer_w ** 4) - (inner_w ** 4)) / 12.0
        elif profile == "H-Beam":
            w = dim_w
            h = dim_w
            t_f = dim_t
            t_w = dim_t
            inner_h = max(0.0, h - 2.0 * t_f)
            inner_w = max(0.0, w - t_w)
            area = (w * h) - (inner_w * inner_h)
            inertia = ((w * (h ** 3)) - (inner_w * (inner_h ** 3))) / 12.0
        elif profile == "Solid Bar":
            radius = dim_w / 2.0
            area = math.pi * (radius ** 2)
            inertia = (math.pi * (dim_w ** 4)) / 64.0
        else:
            area = 0.0001
            inertia = 1e-8
            
        return area, inertia
    
    @staticmethod
    def calculate_buckling_load(material_name, inertia, length_m):
        """
        Calculate Euler buckling critical load in Newtons.
        
        Args:
            material_name: Material type (Steel, Aluminum, Titanium)
            inertia: Moment of inertia in m⁴
            length_m: Beam length in meters
            
        Returns:
            float: Critical buckling load in N, or inf if length <= 0
        """
        if length_m <= 0:
            return float('inf')
        modulus = MaterialManager.get_modulus(material_name)
        return (math.pi ** 2) * modulus * inertia / (length_m ** 2)
    
    @staticmethod
    def get_next_profile(current_profile):
        """Cycle to next profile type in rotation."""
        try:
            idx = PROFILE_TYPES.index(current_profile)
            return PROFILE_TYPES[(idx + 1) % len(PROFILE_TYPES)]
        except ValueError:
            return PROFILE_TYPES[0]
    
    @staticmethod
    def adjust_dimensions(profile, dim_w, dim_t, delta):
        """
        Adjust beam dimensions by delta.
        Solid Bar only adjusts width, tubed profiles adjust both.
        
        Args:
            profile: Profile type
            dim_w: Current width/diameter
            dim_t: Current thickness
            delta: Change amount (+ increases, - decreases)
            
        Returns:
            tuple: (new_dim_w, new_dim_t)
        """
        if profile == "Solid Bar":
            new_w = max(0.01, min(0.32, dim_w + delta))
            return new_w, dim_t
        else:
            new_w = max(0.01, min(0.32, dim_w + delta))
            if delta > 0:
                new_t = max(0.002, min(new_w * 0.4, dim_t + (delta * 0.15)))
            else:
                new_t = max(0.002, min(new_w * 0.4, dim_t))
            return new_w, new_t
    
    @staticmethod
    def get_next_material(current_material):
        """Cycle to next material type in rotation (Steel -> Aluminum -> Titanium -> Steel)."""
        materials = ["Steel", "Aluminum", "Titanium"]
        try:
            idx = materials.index(current_material)
            return materials[(idx + 1) % len(materials)]
        except ValueError:
            return "Steel"
