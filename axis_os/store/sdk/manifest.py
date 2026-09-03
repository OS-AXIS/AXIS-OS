"""
AXIS-OS Robotics Play Store — App Manifest
============================================
Contrat déclaratif que toute app doit fournir avant installation.

Licence : Apache 2.0 (open-source — SDK public)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


class Permission(Enum):
    ACTUATORS_READ       = auto()
    ACTUATORS_WRITE_ARMS = auto()
    ACTUATORS_WRITE_LEGS = auto()
    ACTUATORS_WRITE_ALL  = auto()
    SENSORS_IMU          = auto()
    SENSORS_FORCE_TORQUE = auto()
    SENSORS_CAMERA       = auto()
    SENSORS_MICROPHONE   = auto()
    WATCHDOG_READ        = auto()
    EMERGENCY_STOP       = auto()
    FLEET_BROADCAST      = auto()
    NETWORK_CLOUD        = auto()
    NETWORK_LOCAL        = auto()


class AppCategory(Enum):
    LOGISTICS     = "Logistique"
    SECURITY      = "Sécurité"
    MAINTENANCE   = "Maintenance"
    HEALTHCARE    = "Santé"
    MANUFACTURING = "Fabrication"
    RESEARCH      = "Recherche"
    UTILITIES     = "Utilitaires"


class PricingModel(Enum):
    FREE           = "free"
    ONE_TIME       = "one_time"
    SUBSCRIPTION   = "sub_month"
    PAY_PER_MISSION = "per_mission"


@dataclass
class HardwareRequirements:
    min_dof:              int        = 0
    requires_arms:        bool       = False
    requires_legs:        bool       = False
    requires_imu:         bool       = False
    requires_force_torque: bool      = False
    requires_camera:      bool       = False
    compatible_models:    list[str]  = field(default_factory=list)


@dataclass
class AppManifest:
    app_id:        str
    name:          str
    version:       str
    author:        str
    author_email:  str
    description:   str
    category:      AppCategory

    permissions:   list[Permission]        = field(default_factory=list)
    hardware:      HardwareRequirements    = field(default_factory=HardwareRequirements)

    pricing_model:  PricingModel = PricingModel.FREE
    base_price_eur: float        = 0.0

    max_velocity_ratio:        float = 0.8
    requires_human_supervision: bool = True

    icon_url:      Optional[str] = None
    website_url:   Optional[str] = None
    support_email: Optional[str] = None

    COMMISSION_RATE = 0.30

    @property
    def store_price_eur(self) -> float:
        if self.pricing_model == PricingModel.FREE: return 0.0
        return round(self.base_price_eur / (1 - self.COMMISSION_RATE), 2)

    @property
    def developer_revenue_eur(self) -> float:
        return self.base_price_eur

    @property
    def platform_commission_eur(self) -> float:
        return round(self.store_price_eur - self.base_price_eur, 2)

    def has_permission(self, perm: Permission) -> bool:
        return perm in self.permissions

    def validate(self) -> list[str]:
        errors = []
        if not self.app_id or "." not in self.app_id:
            errors.append("app_id doit être en format 'com.author.name'")
        if self.base_price_eur < 0:
            errors.append("base_price_eur ne peut pas être négatif")
        if self.pricing_model != PricingModel.FREE and self.base_price_eur == 0:
            errors.append("Un modèle payant doit avoir un base_price_eur > 0")
        if not (0.0 < self.max_velocity_ratio <= 1.0):
            errors.append("max_velocity_ratio doit être dans ]0.0, 1.0]")
        return errors
